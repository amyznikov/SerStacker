/*
 * QImageEditor.cc
 *
 *  Created on: May 2, 2022
 *      Author: amyznikov
 */

#include "QImageEditor.h"

namespace {
  enum DISPLAY_TYPE {
    DISPLAY_PIXEL_VALUE,
  };
}

template<>
const c_enum_member* members_of<DISPLAY_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { DISPLAY_PIXEL_VALUE, "VALUE" },
      { DISPLAY_PIXEL_VALUE }
  };

  return members;
}

namespace qserstacker {


QImageEditorDisplaySettings::QImageEditorDisplaySettings(QObject * parent)
{
  Base::displayType_ = DISPLAY_PIXEL_VALUE;
  addDisplay(displayParams_, DISPLAY_PIXEL_VALUE, -1, -1);
}

const c_enum_member * QImageEditorDisplaySettings::displayTypes() const
{
  return members_of<DISPLAY_TYPE>();
}

void QImageEditorDisplaySettings::loadParameters()
{
  Base::loadParameters("QImageEditorDisplaySettings");
}

void QImageEditorDisplaySettings::saveParameters() const
{
  Base::saveParameters("QImageEditorDisplaySettings");
}


QImageEditor::QImageEditor(QWidget * parent) :
  Base(parent),
  displaySettings_(this),
  display_(&displaySettings_, this)
{
  setDisplayFunction(&display_);
}

QMtfImageDisplaySettings * QImageEditor::displaySettings()
{
  return &displaySettings_;
}

const QMtfImageDisplaySettings * QImageEditor::displaySettings() const
{
  return &displaySettings_;
}


} /* namespace qserstacker */
