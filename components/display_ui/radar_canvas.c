#include "radar_canvas.h"

static lv_obj_t *canvas = NULL;

lv_obj_t *radar_canvas_create(lv_obj_t *parent)
{
    canvas = lv_obj_create(parent);

    lv_obj_remove_style_all(canvas);

    lv_obj_set_size(canvas, 220, 220);

    lv_obj_center(canvas);

    lv_obj_set_style_border_width(canvas, 1, 0);
    lv_obj_set_style_border_color(
        canvas,
        lv_palette_main(LV_PALETTE_ORANGE),
        0);

    lv_obj_set_style_bg_opa(canvas, LV_OPA_TRANSP, 0);

    return canvas;
}

void radar_canvas_redraw(void)
{
    if(canvas)
    {
        lv_obj_invalidate(canvas);
    }
}
