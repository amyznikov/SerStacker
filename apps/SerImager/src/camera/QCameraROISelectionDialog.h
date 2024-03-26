/*
 * QCameraROISelectionDialog.h
 *
 *  Created on: Oct 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCaptureROISelectionDialog_h__
#define __QCaptureROISelectionDialog_h__

#include <gui/widgets/QSettingsWidget.h>

struct QCameraROI
{
  QString name;
  QRect rect;

  QCameraROI()
  {
  }

  QCameraROI(const QString & name, const QRect & rc) :
      name(name),
      rect(rc)
  {
  }

  QString toQString() const
  {
    QString text = name.isEmpty() ? "" :
        qsprintf("%s : ", name.toUtf8().constData());

    if( rect.x() < 0 || rect.y() < 0 ) {
      text.append(qsprintf("%dx%d", rect.width(), rect.height()));
    }
    else {
      text.append(qsprintf("%d,%d %dx%d", rect.x(), rect.y(),
          rect.width(), rect.height()));
    }
    return text;
  }

  static bool registerMetatype();
//  {
//    if( !metatype_registered_ ) {
//      qRegisterMetaTypeStreamOperators<QCameraROI>("QCameraROI");
//      qRegisterMetaTypeStreamOperators<QList<QCameraROI>>("QList<QCameraROI>");
//      metatype_registered_ = true;
//    }
//    return metatype_registered_;
//  }

protected:
  static bool metatype_registered_;
};

Q_DECLARE_METATYPE(QCameraROI)

inline QDataStream& operator << (QDataStream & out, const QCameraROI & v)
{
  return (out << v.name << v.rect);
}

inline QDataStream& operator >> (QDataStream & in, QCameraROI & v)
{
  return (in >> v.name >> v.rect);
}

inline QDataStream& operator << (QDataStream & out, const QList<QCameraROI> & list)
{
  out << list.size();
  for( int i = 0, n = list.size(); i < n; ++i ) {
    out << list[i];
  }
  return out;
}

inline QDataStream& operator >> (QDataStream & in, QList<QCameraROI> & list)
{
  list.clear();

  int n = 0;
  if( (in >> n).status() == QDataStream::Ok ) {
    for( int i = 0; i < n; ++i ) {
      QCameraROI p;
      if( (in >> p).status() != QDataStream::Ok ) {
        break;
      }
      list.append(p);
    }
  }
  return in;
}



class QCameraROISelectionDialog :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QCameraROISelectionDialog ThisClass;
  typedef QDialog Base;
  typedef std::function<bool(QRect & rc)> ROIValidator;

  QCameraROISelectionDialog(QWidget * parent = nullptr);

  void setROIList(const QList<QCameraROI> & list);
  const QList<QCameraROI> & roiList() const;

  void setValidator(const ROIValidator & v);
  const ROIValidator & validator() const;

  bool hasChanges() const;

protected:
  void populateROISelectionCombo();
  void setHasChanges(bool v);
  static int findROIByName(const QList<QCameraROI> & list, const QString & name);

protected:
  QComboBox * roiSelection_ctl = nullptr;
  QSettingsWidget * roiOptions_ctl = nullptr;
  QLineEditBox * roiName_ctl_ = nullptr;
  QNumericBox * roiRect_ctl = nullptr;
  QToolButton * addRoi_ctl = nullptr;
  QToolButton * deleteRoi_ctl = nullptr;
  QList<QCameraROI> roiList_;
  ROIValidator validator_;
  bool updatingControls_ = false;
  bool hasChanges_ = false;
};



#endif /* __QCaptureROISelectionDialog_h__ */
