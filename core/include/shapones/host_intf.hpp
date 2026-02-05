#ifndef SHAPONES_HOST_INTF_HPP
#define SHAPONES_HOST_INTF_HPP

#include "shapones/common.hpp"
#include "shapones/fsys.hpp"

namespace nes {

result_t ram_alloc(size_t size, void **out_ptr);
void ram_free(void *ptr);

result_t spinlock_init(int id);
void spinlock_deinit(int id);
void spinlock_get(int id);
void spinlock_release(int id);

result_t semaphore_init(int id);
void semaphore_deinit(int id);
void semaphore_take(int id);
bool semaphore_try_take(int id);
void semaphore_give(int id);

result_t load_ines(const char *path, const uint8_t **out_ines,
                   size_t *out_size);
void unload_ines();

namespace fsys {

result_t mount();
void unmount();
void get_ines_dir(char *out_path);
void get_config_dir(char *out_path);
result_t enum_files(const char *path, enum_files_cb_t callback);
bool exists(const char *path);
result_t open(const char *path, bool write, void **handle);
void close(void *handle);
result_t seek(void *handle, size_t offset);
bool eof(void *handle);
result_t read(void *handle, uint8_t *buff, size_t size);
result_t write(void *handle, const uint8_t *buff, size_t size);
result_t size(void *handle, size_t *out_size);
result_t remove(const char *path);
result_t make_dir(const char *path);

}  // namespace fsys

uint64_t get_time_us();

}  // namespace nes

#endif
