#include <pico/sem.h>
#include <pico/stdlib.h>

#include <ff.h>
//----
#include <diskio.h>

#include <shapones/host_intf.hpp>

namespace shapones {

static constexpr uint32_t MAX_INES_SIZE = (128 + 1) * 1024;

static uint8_t ines_sram[MAX_INES_SIZE] = {0xEA};

static spin_lock_t *lock_hws[NUM_SPINLOCKS];
static uint32_t lock_irqs[NUM_SPINLOCKS];

static semaphore_t semaphores[NUM_SEMAPHORES];

FATFS fs;

result_t load_ines(const char *path, const uint8_t **out_ines,
                   size_t *out_size) {
  result_t res = result_t::SUCCESS;
  FIL fil;
  FRESULT fr;
  size_t size = 0;

  fr = f_open(&fil, path, FA_READ);
  if (fr) {
    SHAPONES_RET_ERR(result_t::ERR_FS_OPEN_FAILED);
  }

  do {
    size = f_size(&fil);
    if (size > MAX_INES_SIZE) {
      SHAPONES_BRK_ERR(res, result_t::ERR_INES_TOO_LARGE);
    }

    UINT sz;
    fr = f_read(&fil, ines_sram, size, &sz);
    if (fr) {
      SHAPONES_BRK_ERR(res, result_t::ERR_FS_READ_FAILED);
    }

  } while (0);

  f_close(&fil);

  *out_ines = ines_sram;
  *out_size = size;

  return res;
}

void unload_ines() {}

result_t ram_alloc(size_t size, void **out_ptr) {
  *out_ptr = malloc(size);
  if (!*out_ptr) {
    SHAPONES_RET_ERR(result_t::ERR_RAM_ALLOC_FAILED);
  }
  return result_t::SUCCESS;
}

void ram_free(void *ptr) { free(ptr); }

result_t spinlock_init(int id) {
  lock_hws[id] = spin_lock_init(id);
  spin_lock_claim(id);
  return result_t::SUCCESS;
}
void spinlock_deinit(int id) { spin_lock_unclaim(id); }
void spinlock_get(int id) { lock_irqs[id] = spin_lock_blocking(lock_hws[id]); }
void spinlock_release(int id) { spin_unlock(lock_hws[id], lock_irqs[id]); }

result_t semaphore_init(int id) {
  sem_init(&semaphores[id], 1, 1);
  return result_t::SUCCESS;
}
void semaphore_deinit(int id) {
  // No deinitialization needed for pico semaphores
}
void semaphore_take(int id) { sem_acquire_blocking(&semaphores[id]); }
bool semaphore_try_take(int id) { return sem_try_acquire(&semaphores[id]); }
void semaphore_give(int id) { sem_release(&semaphores[id]); }

namespace fsys {

result_t mount() {
  FRESULT fres = f_mount(&fs, "/", 0);
  if (fres) {
    SHAPONES_RET_ERR(result_t::ERR_FS_NO_DISK);
  }
  return result_t::SUCCESS;
}

void unmount() {
  // No unmount operation needed for pico FATFS
}

void get_ines_dir(char *out_path) { strcpy(out_path, "/"); }

void get_config_dir(char *out_path) { strcpy(out_path, "/.shapones/"); }

result_t enum_files(const char *path, enum_files_cb_t callback) {
  DIR dobj;
  FILINFO finfo;
  FRESULT fres = f_findfirst(&dobj, &finfo, path, "*");
  while (fres == FR_OK && finfo.fname[0]) {
    file_info_t info;
    info.is_dir = (finfo.fattrib & AM_DIR) != 0;
    info.name = finfo.fname;
    if (!callback(info)) {
      break;
    }
    fres = f_findnext(&dobj, &finfo);
  }
  f_closedir(&dobj);
  if (fres) {
    SHAPONES_RET_ERR(result_t::ERR_FS_ENUM_FAILED);
  }
  return result_t::SUCCESS;
}

bool exists(const char *path) {
  FILINFO finfo;
  FRESULT fres = f_stat(path, &finfo);
  return (fres == FR_OK);
}

result_t open(const char *path, bool write, void **handle) {
  FIL *fil = new FIL();
  FRESULT fres = f_open(fil, path, write ? (FA_READ | FA_WRITE) : FA_READ);
  if (fres) {
    delete fil;
    SHAPONES_RET_ERR(result_t::ERR_FS_OPEN_FAILED);
  }
  *handle = fil;
  return result_t::SUCCESS;
}

void close(void *handle) {
  FIL *fil = static_cast<FIL *>(handle);
  f_close(fil);
  delete fil;
}

result_t seek(void *handle, size_t offset) {
  FIL *fil = static_cast<FIL *>(handle);
  FRESULT fres = f_lseek(fil, offset);
  if (fres) {
    SHAPONES_RET_ERR(result_t::ERR_FS_SEEK_FAILED);
  }
  return result_t::SUCCESS;
}

bool eof(void *handle) {
  FIL *fil = static_cast<FIL *>(handle);
  return (fil->fptr >= f_size(fil));
}

result_t read(void *handle, uint8_t *buff, size_t size) {
  FIL *fil = static_cast<FIL *>(handle);
  UINT sz;
  FRESULT fres = f_read(fil, buff, size, &sz);
  if (fres) {
    SHAPONES_RET_ERR(result_t::ERR_FS_READ_FAILED);
  }
  if (sz != size) {
    SHAPONES_RET_ERR(result_t::ERR_FS_READ_FAILED);
  }
  return result_t::SUCCESS;
}

result_t write(void *handle, const uint8_t *buff, size_t size) {
  FIL *fil = static_cast<FIL *>(handle);
  UINT sz;
  FRESULT fres = f_write(fil, buff, size, &sz);
  if (fres) {
    SHAPONES_RET_ERR(result_t::ERR_FS_WRITE_FAILED);
  }
  if (sz != size) {
    SHAPONES_RET_ERR(result_t::ERR_FS_WRITE_FAILED);
  }
  return result_t::SUCCESS;
}

result_t size(void *handle, size_t *out_size) {
  FIL *fil = static_cast<FIL *>(handle);
  *out_size = f_size(fil);
  return result_t::SUCCESS;
}

result_t remove(const char *path) {
  FRESULT fres = f_unlink(path);
  if (fres) {
    SHAPONES_RET_ERR(result_t::ERR_FS_DELETE_FAILED);
  }
  return result_t::SUCCESS;
}

result_t make_dir(const char *path) {
  FRESULT fres = f_mkdir(path);
  if (fres) {
    SHAPONES_RET_ERR(result_t::ERR_FS_WRITE_FAILED);
  }
  return result_t::SUCCESS;
}

}  // namespace fsys

uint64_t get_time_us() { return to_us_since_boot(get_absolute_time()); }

}  // namespace shapones