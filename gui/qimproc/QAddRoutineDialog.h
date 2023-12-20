/*
 * QAddRoutineDialog.h
 *
 *  Created on: Aug 7, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QAddRoutineDialog_h__
#define __QAddRoutineDialog_h__

#include <QtWidgets/QtWidgets>
#include <core/improc/c_image_processor.h>

class QAddRoutineDialog :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QAddRoutineDialog ThisClass;
  typedef QDialog Base;

  QAddRoutineDialog(QWidget * parent = nullptr);

  const c_image_processor_routine::class_factory * selectedClassFactory() const;

protected Q_SLOTS:
  void populateClassList();
  void onFilterTextChanged(const QString &);
  void onClassListCurrentItemChanged(QListWidgetItem *, QListWidgetItem *);
  void onOkClicked();


protected:
  const c_image_processor_routine::class_factory * selectedClassFactory_ = nullptr;
  QComboBox * filter_ctl = nullptr;
  QListWidget * classlist_ctl = nullptr;
  QLabel * tooltip_ctl = nullptr;
  QPushButton * ok_ctl = nullptr;
  QPushButton * cancel_ctl = nullptr;
};

#endif /* __QAddRoutineDialog_h__ */
