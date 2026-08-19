#pragma once

#include "lvgl.h"

lv_obj_t *radar_canvas_create(lv_obj_t *parent);

void radar_canvas_redraw(void);
