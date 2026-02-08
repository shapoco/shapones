#pragma GCC optimize("O2")

#include <M5Unified.h>
#include "FS.h"
#include "SD.h"
#include <esp_partition.h>
#include <spi_flash_mmap.h>

#include "shapones_config.hpp"
#include "shapones_core.h"

static uint8_t ines_sram[SHAPONES_MAX_INES_IN_SRAM];
static uint8_t *ines_psram = nullptr;

static const esp_partition_t *ines_partition = nullptr;
static spi_flash_mmap_handle_t mmap_handle = 0;

static spinlock_t spinlocks[shapones::NUM_SPINLOCKS];
static SemaphoreHandle_t semaphores[shapones::NUM_SEMAPHORES];

void init_host_impl() {
  ines_partition = esp_partition_find_first(
    ESP_PARTITION_TYPE_DATA,
    ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
    "spiffs");
  if (!ines_partition) {
    SHAPONES_PRINTF("*Warning: SPIFFS Partition Not Found.");
  }
}

static shapones::result_t load_ines_to_flash(File &f, size_t file_size, const uint8_t **out_ines) {
  shapones::result_t res = shapones::result_t::SUCCESS;
  esp_err_t esp_err;

  constexpr int CHUNK_SIZE = 4096;

  //uint8_t *buff = new uint8_t[CHUNK_SIZE];
  uint8_t *buff = ines_sram;

  mmap_handle = 0;

  do {
    int bar_w = M5.Display.width() * 3 / 4;
    int bar_h = bar_w / 10;
    int bar_x = (M5.Display.width() - bar_w) / 2;
    int bar_y = (M5.Display.height() - bar_h) / 2;
    M5.Display.fillRect(bar_x - 2, bar_y - 2, bar_w + 4, bar_h + 4, 0x0000);
    M5.Display.fillRect(bar_x, bar_y, 1, bar_h, 0xFFFF);

    SHAPONES_PRINTF("Loading iNES to Flash...\n");
    SHAPONES_PRINTF("  Erasing...\n");
    size_t erase_size = (file_size + SPI_FLASH_SEC_SIZE - 1) & ~(SPI_FLASH_SEC_SIZE - 1);
    esp_err = esp_partition_erase_range(ines_partition, 0, erase_size);
    if (esp_err != ESP_OK) {
      res = shapones::result_t::ERR_FLASH_ERASE_FAILED;
      break;
    }

    SHAPONES_PRINTF("  Programming...\n");
    size_t offset = 0;
    while (f.available()) {
      size_t bytes_read = f.read(buff, CHUNK_SIZE);
      esp_err = esp_partition_write(ines_partition, offset, buff, bytes_read);
      if (esp_err != ESP_OK) {
        res = shapones::result_t::ERR_FLASH_PROGRAM_FAILED;
        break;
      }
      offset += bytes_read;
      M5.Display.fillRect(bar_x, bar_y, (int)(bar_w * offset / file_size), bar_h, 0xFFFF);
    }

    SHAPONES_PRINTF("  Mapping to Memory...\n");
    esp_err = esp_partition_mmap(
      ines_partition, 0, ines_partition->size,
      ESP_PARTITION_MMAP_DATA, (const void **)out_ines, &mmap_handle);
    if (esp_err != ESP_OK) {
      res = shapones::result_t::ERR_MMAP_FAILED;
      break;
    }
  } while (0);

  //delete[] buff;

  return shapones::result_t::SUCCESS;
}

namespace shapones {

result_t ram_alloc(size_t size, void **out_ptr) {
  void *ptr = heap_caps_malloc(size, MALLOC_CAP_DMA);
  if (!ptr) {
    ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!ptr) {
      return result_t::ERR_RAM_ALLOC_FAILED;
    }
  }
  *out_ptr = ptr;
  return result_t::SUCCESS;
}

void ram_free(void *ptr) {
  heap_caps_free(ptr);
}

result_t spinlock_init(int id) {
  spinlock_initialize(&spinlocks[id]);
  return result_t::SUCCESS;
}
void spinlock_deinit(int id) {}
void spinlock_get(int id) {
  taskENTER_CRITICAL(&spinlocks[id]);
}
void spinlock_release(int id) {
  taskEXIT_CRITICAL(&spinlocks[id]);
}

result_t semaphore_init(int id) {
  semaphores[id] = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphores[id]);
  return result_t::SUCCESS;
}
void semaphore_deinit(int id) {}
void semaphore_take(int id) {
  while (xSemaphoreTake(semaphores[id], 0) == pdFALSE) {
  }
}
bool semaphore_try_take(int id) {
  return (xSemaphoreTake(semaphores[id], 0) == pdTRUE);
}
void semaphore_give(int id) {
  xSemaphoreGive(semaphores[id]);
}

result_t load_ines(const char *path, const uint8_t **out_ines,
                   size_t *out_size) {
  result_t res = result_t::SUCCESS;
  esp_err_t esp_err;

  unload_ines();

  SHAPONES_PRINTF("Loading iNES: '%s'\n", path);
  File f = SD.open(path, FILE_READ);
  if (!f) {
    return result_t::ERR_FS_OPEN_FAILED;
  }

  size_t file_size = f.size();
  SHAPONES_PRINTF("size: %d\n", (int)file_size);

  uint8_t *ines_ptr = nullptr;

  do {
    if (file_size <= SHAPONES_MAX_INES_IN_SRAM) {
      // Load to SRAM
      ines_ptr = ines_sram;
    } 
    else {
      // Load to PSRAM
      ines_psram = (uint8_t *)heap_caps_malloc(file_size, MALLOC_CAP_SPIRAM);
      ines_ptr = ines_psram;
    }

    if (ines_ptr) {
      // Load to RAM
      size_t s = f.read(ines_ptr, file_size);
      if (s != file_size) {
        res = result_t::ERR_FS_READ_FAILED;
        break;
      }
    } else {
      // Load to Flash
      res = load_ines_to_flash(f, file_size, (const uint8_t **)&ines_ptr);
      if (res != result_t::SUCCESS) {
        break;
      }
    }

  } while (0);

  f.close();

  *out_ines = ines_ptr;
  *out_size = file_size;

  return res;
}

void unload_ines() {
  if (ines_psram) {
    heap_caps_free(ines_psram);
    ines_psram = nullptr;
  }
  if (mmap_handle) {
    spi_flash_munmap(mmap_handle);
    mmap_handle = 0;
  }
}

uint64_t get_time_us() {
  uint64_t t = micros();
  return t;
}

namespace fsys {

static File file_handle;

result_t mount() {
  if (SD.begin(TF_CS_PIN, SPI, 10000000)) {
    return result_t::SUCCESS;
  } else {
    Serial.printf("No Disk.\n");
    vTaskDelay(1000);
    return result_t::ERR_FS_NO_DISK;
  }
}

void unmount() {
  SD.end();
}

void get_ines_dir(char *out_path) {
  strncpy(out_path, "/", MAX_PATH_LENGTH);
}

void get_config_dir(char *out_path) {
  strncpy(out_path, "/.shapones", MAX_PATH_LENGTH);
}

result_t enum_files(const char *path, enum_files_cb_t callback) {
  bool is_dir;
  File root = SD.open(path);
  while (1) {
    file_info_t fi;
    String filename = root.getNextFileName(&fi.is_dir);
    int sep = filename.lastIndexOf('/');
    if (sep >= 0) {
      filename = filename.substring(sep + 1);
    }
    if (filename == "") break;
    fi.name = (char *)filename.c_str();
    if (!callback(fi)) break;
  }
  root.close();
  return result_t::SUCCESS;
}

bool exists(const char *path) {
  return SD.exists(path);
}

result_t open(const char *path, bool write, void **handle) {
  file_handle = SD.open(path, write ? FILE_WRITE : FILE_READ);
  if (!file_handle) {
    Serial.printf("Open failed: %s\n", path);
    return result_t::ERR_FS_OPEN_FAILED;
  }
  *handle = &file_handle;
  return result_t::SUCCESS;
}

void close(void *handle) {
  File *f = (File *)handle;
  f->close();
}

result_t seek(void *handle, size_t offset) {
  File *f = (File *)handle;
  if (f->seek(offset)) {
    return result_t::SUCCESS;
  } else {
    return result_t::ERR_FS_SEEK_FAILED;
  }
}

result_t size(void *handle, size_t *out_size) {
  File *f = (File *)handle;
  *out_size = f->size();
  return result_t::SUCCESS;
}

result_t read(void *handle, uint8_t *buff, size_t size) {
  File *f = (File *)handle;
  size_t s = f->read(buff, size);
  if (s == size) {
    return result_t::SUCCESS;
  } else {
    return result_t::ERR_FS_READ_FAILED;
  }
}

result_t write(void *handle, const uint8_t *buff, size_t size) {
  File *f = (File *)handle;
  size_t s = f->write(buff, size);
  if (s == size) {
    return result_t::SUCCESS;
  } else {
    return result_t::ERR_FS_WRITE_FAILED;
  }
}

result_t remove(const char *path) {
  if (SD.remove(path)) {
    return result_t::SUCCESS;
  } else {
    return result_t::ERR_FS_DELETE_FAILED;
  }
}

}

}  // namespace shapones
