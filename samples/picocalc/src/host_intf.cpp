#include <shapones/host_intf.hpp>

#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "shapones/xiao/lock.h"
#include "shapones/xiao/ram.h"
#include "shapones/xiao/timer.h"

#include <ff.h>
//----
#include <diskio.h>

namespace shapones {

// PicoCalc: the whole ROM goes in RAM (no frame buffer is kept, so a few
// hundred KB are free); refuse files that wouldn't leave room to run.
static uint8_t *ines_ram = nullptr;
static constexpr size_t MAX_INES_SIZE = 384 * 1024 + 16 + 512;

static xiao_spin_lock_t spinlocks[NUM_SPINLOCKS];
static xiao_semaphore_t semaphores[NUM_SEMAPHORES];

FATFS fs;

void unload_ines() {
  free(ines_ram);
  ines_ram = nullptr;
}

result_t load_ines(const char *path, const uint8_t **out_ines,
                   size_t *out_size) {
  result_t res = result_t::SUCCESS;
  FIL fil;
  size_t size = 0;

  unload_ines();
  if (f_open(&fil, path, FA_READ)) {
    SHAPONES_RET_ERR(result_t::ERR_FS_OPEN_FAILED);
  }
  do {
    size = f_size(&fil);
    if (size > MAX_INES_SIZE) {
      SHAPONES_BRK_ERR(res, result_t::ERR_INES_TOO_LARGE);
    }
    ines_ram = (uint8_t *)malloc(size);
    if (!ines_ram) {
      SHAPONES_BRK_ERR(res, result_t::ERR_INES_TOO_LARGE);
    }
    UINT sz;
    if (f_read(&fil, ines_ram, size, &sz) || sz != size) {
      unload_ines();
      SHAPONES_BRK_ERR(res, result_t::ERR_FS_READ_FAILED);
    }
  } while (0);
  f_close(&fil);

  *out_ines = ines_ram;
  *out_size = size;
  return res;
}

result_t ram_alloc(size_t size, void **out_ptr) {
  *out_ptr = xiao_malloc(size, true);
  if (!*out_ptr) {
    *out_ptr = xiao_malloc(size, false);
    if (!*out_ptr) {
      SHAPONES_RET_ERR(result_t::ERR_RAM_ALLOC_FAILED);
    }
  }
  return result_t::SUCCESS;
}

void ram_free(void *ptr) { xiao_free(ptr); }

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

// ROMs live in /nes if that folder exists, otherwise in the root
void get_ines_dir(char *out_path) {
  FILINFO fi;
  strcpy(out_path, f_stat("/nes", &fi) == FR_OK && (fi.fattrib & AM_DIR) ? "/nes/" : "/");
}

void get_config_dir(char *out_path) { strcpy(out_path, "/nes-saves/"); }

result_t enum_files(const char *path, enum_files_cb_t callback) {
  DIR dobj;
  FILINFO finfo;
  FRESULT fres = f_findfirst(&dobj, &finfo, path, "*");
  while (fres == FR_OK && finfo.fname[0]) {
    file_info_t info;
    info.is_dir = (finfo.fattrib & AM_DIR) != 0;
    info.name = finfo.fname;
    // PicoCalc: list only folders and .nes files (the card's root also
    // holds programs, screenshots and settings)
    size_t n = strlen(finfo.fname);
    bool nes = n > 4 && strcasecmp(finfo.fname + n - 4, ".nes") == 0;
    bool hidden = finfo.fname[0] == '.' || (finfo.fattrib & (AM_HID | AM_SYS));
    if ((info.is_dir || nes) && !hidden && !callback(info)) {
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
  int mode;
  if (write) {
    mode = FA_READ | FA_WRITE;
    if (!exists(path)) {
      mode |= FA_CREATE_NEW;
    }
  } else {
    mode = FA_READ;
  }

  FIL *fil = new FIL();
  FRESULT fres = f_open(fil, path, mode);
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
