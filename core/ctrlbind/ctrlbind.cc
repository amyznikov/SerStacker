/*
 * ctrlbind.cc
 *
 *  Created on: Apr 3, 2024
 *      Author: amyznikov
 */

#include "ctrlbind.h"
#include <core/feature2d/feature_extraction.h>

static ctrlbind_copy_to_clipboard_callback copy_to_clipboard_callback_instance;
static ctrlbind_update_roi_callback update_roi_callback_instance;
static ctrlbind_get_clipboard_text_callback get_clipboard_text_callback_instance;

void set_ctrlbind_copy_to_clipboard_callback(const ctrlbind_copy_to_clipboard_callback & fn)
{
  copy_to_clipboard_callback_instance = fn;
}

const ctrlbind_copy_to_clipboard_callback & get_ctrlbind_copy_to_clipboard_callback()
{
  return copy_to_clipboard_callback_instance;
}

void set_ctrlbind_get_clipboard_text_callback(const ctrlbind_get_clipboard_text_callback & fn)
{
  get_clipboard_text_callback_instance = fn;
}

const ctrlbind_get_clipboard_text_callback & get_ctrlbind_get_clipboard_text_callback()
{
  return get_clipboard_text_callback_instance;
}

void set_ctrlbind_update_roi_callback(const ctrlbind_update_roi_callback & fn)
{
  update_roi_callback_instance = fn;
}

const ctrlbind_update_roi_callback & get_ctrlbind_update_roi_callback()
{
  return update_roi_callback_instance;
}
