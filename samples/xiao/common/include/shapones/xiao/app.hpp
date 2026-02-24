#ifndef SHAPONES_XIAO_APP_HPP
#define SHAPONES_XIAO_APP_HPP

namespace shapones::xiao {

void app_init();
void cpu_service();
void ppu_service();
void shutdown();

}  // namespace shapones::xiao

#endif