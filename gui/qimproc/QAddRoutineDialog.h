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

class QAddRoutineDialog
    : public QDialog
{
  Q_OBJECT;
public:
  typedef QAddRoutineDialog ThisClass;
  typedef QDialog Base;

  QAddRoutineDialog(QWidget * parent = Q_NULLPTR);

  const c_image_processor_routine::class_factory * selectedClassFactory() const;

protected slots:
  void populateClassList();
  void onFilterTextChanged(const QString &);
  void onClassListCurrentItemChanged(QListWidgetItem *, QListWidgetItem *);
  void onOkClicked();


protected:
  const c_image_processor_routine::class_factory * selectedClassFactory_ = Q_NULLPTR;
  QComboBox * filter_ctl = Q_NULLPTR;
  QListWidget * classlist_ctl = Q_NULLPTR;
  QLabel * tooltip_ctl = Q_NULLPTR;
  QPushButton * ok_ctl = Q_NULLPTR;
  QPushButton * cancel_ctl = Q_NULLPTR;
};

#endif /* __QAddRoutineDialog_h__ */
