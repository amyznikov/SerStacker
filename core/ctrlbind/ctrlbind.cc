/*
 * ctrlbind.cc
 *
 *  Created on: Apr 3, 2024
 *      Author: amyznikov
 */

#include "ctrlbind.h"
#include <core/feature2d/feature_extraction.h>

static ctrlbind_copy_to_clipboard_callback callback_instance;

void set_ctrlbind_copy_to_clipboard_callback(const ctrlbind_copy_to_clipboard_callback & fn)
{
  callback_instance = fn;
}

const ctrlbind_copy_to_clipboard_callback & get_ctrlbind_copy_to_clipboard_callback()
{
  return callback_instance;
}
