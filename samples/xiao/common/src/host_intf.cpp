#include <shapones/host_intf.hpp>

#include "shapones/xiao/lock.h"
#include "shapones/xiao/timer.h"

#include <ff.h>
//----
#include <diskio.h>

namespace shapones {

static constexpr uint32_t MAX_INES_SIZE = (64 + 1) * 1024;

static uint8_t ines_sram[MAX_INES_SIZE] = {0xEA};

static xiao_spin_lock_t spinlocks[NUM_SPINLOCKS];
static xiao_semaphore_t semaphores[NUM_SEMAPHORES];

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
  xiao_spinlock_init(&spinlocks[id]);
  return result_t::SUCCESS;
}
void spinlock_deinit(int id) { xiao_spinlock_deinit(&spinlocks[id]); }
void spinlock_get(int id) { xiao_spinlock_get(&spinlocks[id]); }
void spinlock_release(int id) { xiao_spinlock_release(&spinlocks[id]); }

result_t semaphore_init(int id) {
  xiao_semaphore_init(&semaphores[id]);
  return result_t::SUCCESS;
}
void semaphore_deinit(int id) { xiao_semaphore_deinit(&semaphores[id]); }
void semaphore_take(int id) { xiao_semaphore_take(&semaphores[id]); }
bool semaphore_try_take(int id) {
  return xiao_semaphore_try_take(&semaphores[id]);
}
void semaphore_give(int id) { xiao_semaphore_give(&semaphores[id]); }

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

uint64_t get_time_us() { return xiao_get_time_us(); }

}  // namespace shapones
