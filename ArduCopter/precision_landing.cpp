//
// functions to support precision landing 支持精确着陆的功能
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    copter.precland.init();
}

void Copter::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;

    // use range finder altitude if it is valid, else try to get terrain alt //如果有效，请使用测距仪高度，否则尝试获取地形高度
    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.alt_cm;
    } else if (terrain_use()) {
        if (!current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, height_above_ground_cm)) {
            height_above_ground_cm = current_loc.alt;
        }
    }

    copter.precland.update(height_above_ground_cm, rangefinder_alt_ok());
}
#endif
