/*
 * QImageEditor.h
 *
 *  Created on: May 2, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __qserstacker_image_editor_h__
#define __qserstacker_image_editor_h__

#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qimageview/QMtfImageDisplayFunction.h>

namespace qserstacker {

class QImageEditorDisplaySettings :
    public QMtfImageDisplaySettings
{
  Q_OBJECT;
public:
  typedef QImageEditorDisplaySettings ThisClass;
  typedef QMtfImageDisplaySettings Base;

  QImageEditorDisplaySettings(QObject * parent = Q_NULLPTR);
  const c_enum_member * displayTypes() const override;

  void loadParameters() override;
  void saveParameters() const override;
};


class QImageEditor:
    public QImageFileEditor
{
  Q_OBJECT;
public:
  typedef QImageEditor ThisClass;
  typedef QImageFileEditor Base;

  QImageEditor(QWidget * parent = Q_NULLPTR);

  QMtfImageDisplaySettings * displaySettings();
  const QMtfImageDisplaySettings * displaySettings() const;

protected:
  QImageEditorDisplaySettings displaySettings_;
  QMtfImageDisplayFunction display_;
};

} /* namespace qserstacker */

#endif /* __qserstacker_image_editor_h__ */
