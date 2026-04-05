/*
 * ctrlbind.cc
 *
 *  Created on: Apr 3, 2024
 *      Author: amyznikov
 */

#include "ctrlbind.h"

static ctlbind_show_info_text_callback ctlbind_show_info_text_callback_instance;
static ctlbind_copy_to_clipboard_callback copy_to_clipboard_callback_instance;
static ctlbind_get_clipboard_text_callback get_clipboard_text_callback_instance;
static ctlbind_update_roi_callback update_roi_callback_instance;
static ctlbind_get_roi_callback ctrlbind_get_roi_callback_instance;

void set_ctlbind_show_info_text_callback(const ctlbind_show_info_text_callback & fn)
{
  ctlbind_show_info_text_callback_instance = fn;
}

const ctlbind_show_info_text_callback & get_ctlbind_show_info_text_callback()
{
  return ctlbind_show_info_text_callback_instance;
}

void set_ctlbind_copy_to_clipboard_callback(const ctlbind_copy_to_clipboard_callback & fn)
{
  copy_to_clipboard_callback_instance = fn;
}

const ctlbind_copy_to_clipboard_callback & get_ctlbind_copy_to_clipboard_callback()
{
  return copy_to_clipboard_callback_instance;
}

void set_ctlbind_get_clipboard_text_callback(const ctlbind_get_clipboard_text_callback & fn)
{
  get_clipboard_text_callback_instance = fn;
}

const ctlbind_get_clipboard_text_callback & get_ctlbind_get_clipboard_text_callback()
{
  return get_clipboard_text_callback_instance;
}

void set_ctlbind_update_roi_callback(const ctlbind_update_roi_callback & fn)
{
  update_roi_callback_instance = fn;
}

const ctlbind_update_roi_callback & get_ctlbind_update_roi_callback()
{
  return update_roi_callback_instance;
}

void set_ctlbind_get_roi_callback(const ctlbind_get_roi_callback & fn)
{
  ctrlbind_get_roi_callback_instance = fn;
}

const ctlbind_get_roi_callback & get_ctlbind_get_roi_callback()
{
  return ctrlbind_get_roi_callback_instance;
}


