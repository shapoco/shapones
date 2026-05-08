#include <shapones/shapones.hpp>
#include <xiamocon.hpp>

namespace shapones {

constexpr uint32_t MAX_RAM_INES_SIZE = (64 + 1) * 1024;

uint8_t *inesBuff = nullptr;
void *inesMmapHandle = nullptr;

xmc::SpinLock spinlocks[NUM_SPINLOCKS];
xmc::Semaphore semaphores[NUM_SEMAPHORES];

struct FileHandle {
  xmc::fs::File file;
};

result_t load_ines(const char *path, const uint8_t **out_ines,
                   size_t *out_size) {
  XmcStatus xmcSts = XMC_OK;
  size_t size = xmc::fs::getFileSize(path);
  *out_size = size;

  result_t res = result_t::SUCCESS;
  if (size <= MAX_RAM_INES_SIZE || xmcHasSpiRam()) {
    inesBuff = (uint8_t *)xmcMalloc(size, XMC_HEAP_CAP_SPIRAM);
  } else {
    inesBuff = nullptr;
  }

  if (inesBuff) {
    // Load to RAM
    xmc::fs::File file = xmc::fs::openFile(path, xmc::fs::FileMode::READ);
    if (!file->isOpen()) {
      SHAPONES_RET_ERR(result_t::ERR_FS_OPEN_FAILED);
    }
    xmc::speaker::setMuted(true);
    do {
      size_t bytesRead = file->read(inesBuff, size);
      if (bytesRead != size) {
        SHAPONES_BRK_ERR(res, result_t::ERR_FS_READ_FAILED);
      }
    } while (false);
    file->close();
    xmc::speaker::setMuted(false);
    *out_ines = inesBuff;
  } else {
    // Load to Flash
    size_t romBase, romSize;
    xmc::flash::getRange(&romBase, &romSize);
    size_t sectorSize = xmc::flash::getSectorSize();
    size_t numSectors = (size + sectorSize - 1) / sectorSize;
    size_t offset = romBase + romSize - numSectors * sectorSize;

    xmc::speaker::setMuted(true);
    do {
      xmcSts = xmc::fs::copyFileToFlash(path, 0, size, offset, 16 * 1024);
      if (xmcSts != XMC_OK) {
        SHAPONES_BRK_ERR(res, result_t::ERR_FLASH_PROGRAM_FAILED);
      }
      xmcSts = xmc::flash::mmap(offset, size, &inesMmapHandle, out_ines);
      if (xmcSts != XMC_OK) {
        SHAPONES_BRK_ERR(res, result_t::ERR_MMAP_FAILED);
      }
    } while (false);
    xmc::speaker::setMuted(false);
  }
  return res;
}

void unload_ines() {
  if (inesBuff) {
    xmcFree(inesBuff);
    inesBuff = nullptr;
  }
  if (inesMmapHandle) {
    xmc::flash::munmap(inesMmapHandle);
    inesMmapHandle = nullptr;
  }
}

result_t ram_alloc(size_t size, void **out_ptr) {
  *out_ptr = xmcMalloc(size, XMC_HEAP_CAP_DMA);
  if (!*out_ptr) {
    *out_ptr = xmcMalloc(size, XMC_HEAP_CAP_SPIRAM);
    if (!*out_ptr) {
      SHAPONES_RET_ERR(result_t::ERR_RAM_ALLOC_FAILED);
    }
  }
  return result_t::SUCCESS;
}

void ram_free(void *ptr) { xmcFree(ptr); }

result_t spinlock_init(int id) { return result_t::SUCCESS; }
void spinlock_deinit(int id) {}
void spinlock_get(int id) { spinlocks[id].get(); }
void spinlock_release(int id) { spinlocks[id].release(); }

result_t semaphore_init(int id) { return result_t::SUCCESS; }
void semaphore_deinit(int id) {}
void semaphore_take(int id) { semaphores[id].take(); }
bool semaphore_try_take(int id) { return semaphores[id].tryTake(); }
void semaphore_give(int id) { semaphores[id].give(); }

namespace fsys {

result_t mount() {
  XmcStatus xmcSts = xmc::fs::mount();
  if (xmcSts != XMC_OK) {
    SHAPONES_RET_ERR(result_t::ERR_FS_NO_DISK);
  }
  return result_t::SUCCESS;
}

void unmount() { xmc::fs::unmount(); }

void get_ines_dir(char *out_path) { strcpy(out_path, "/"); }

void get_config_dir(char *out_path) { strcpy(out_path, "/.shapones/"); }

result_t enum_files(const char *path, enum_files_cb_t callback) {
  XmcStatus xmcSts = xmc::fs::enumFiles(
      path,
      [](const xmc::fs::FileInfo &info, void *userData) {
        enum_files_cb_t cb = (enum_files_cb_t)userData;
        file_info_t fi;
        fi.is_dir = info.isDirectory;
        fi.name = info.name;
        return cb(fi);
      },
      (void *)callback);
  if (xmcSts != XMC_OK) {
    SHAPONES_RET_ERR(result_t::ERR_FS_ENUM_FAILED);
  }
  return result_t::SUCCESS;
}

bool exists(const char *path) { return xmc::fs::exists(path); }

result_t open(const char *path, bool write, void **handle) {
  xmc::fs::FileMode mode =
      write ? xmc::fs::FileMode::WRITE : xmc::fs::FileMode::READ;
  xmc::fs::File file = xmc::fs::openFile(path, mode);

  if (!file->isOpen()) {
    SHAPONES_RET_ERR(result_t::ERR_FS_OPEN_FAILED);
  }
  FileHandle *fh = new FileHandle();
  fh->file = file;
  *handle = fh;
  return result_t::SUCCESS;
}

void close(void *handle) {
  FileHandle *fh = static_cast<FileHandle *>(handle);
  fh->file->close();
  delete fh;
}

result_t seek(void *handle, size_t offset) {
  FileHandle *fh = static_cast<FileHandle *>(handle);
  XmcStatus xmcSts = fh->file->seek(offset);
  if (xmcSts != XMC_OK) {
    SHAPONES_RET_ERR(result_t::ERR_FS_SEEK_FAILED);
  }
  return result_t::SUCCESS;
}

bool eof(void *handle) {
  FileHandle *fh = static_cast<FileHandle *>(handle);
  return fh->file->eof();
}

result_t read(void *handle, uint8_t *buff, size_t size) {
  FileHandle *fh = static_cast<FileHandle *>(handle);
  size_t bytesRead = fh->file->read(buff, size);
  if (bytesRead != size) {
    SHAPONES_RET_ERR(result_t::ERR_FS_READ_FAILED);
  }
  return result_t::SUCCESS;
}

result_t write(void *handle, const uint8_t *buff, size_t size) {
  FileHandle *fh = static_cast<FileHandle *>(handle);
  size_t bytesWritten = fh->file->write(buff, size);
  if (bytesWritten != size) {
    SHAPONES_RET_ERR(result_t::ERR_FS_WRITE_FAILED);
  }
  return result_t::SUCCESS;
}

result_t size(void *handle, size_t *out_size) {
  FileHandle *fh = static_cast<FileHandle *>(handle);
  *out_size = fh->file->getSize();
  return result_t::SUCCESS;
}

result_t remove(const char *path) {
  XmcStatus xmcSts = xmc::fs::removeFile(path);
  if (xmcSts != XMC_OK) {
    SHAPONES_RET_ERR(result_t::ERR_FS_DELETE_FAILED);
  }
  return result_t::SUCCESS;
}

result_t make_dir(const char *path) {
  XmcStatus xmcSts = xmc::fs::createDirectory(path);
  if (xmcSts != XMC_OK) {
    SHAPONES_RET_ERR(result_t::ERR_FS_WRITE_FAILED);
  }
  return result_t::SUCCESS;
}

}  // namespace fsys

uint64_t get_time_us() { return xmc::getTimeUs(); }

}  // namespace shapones
