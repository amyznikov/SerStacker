/*
 * QAddDataProcRoutineDialogBox.h
 *
 *  Created on: Dec 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QAddDataProcRoutineDialogBox_h__
#define __QAddDataProcRoutineDialogBox_h__

#include <QtWidgets/QtWidgets>
#include <core/dataproc/c_data_frame_processor.h>

class QAddDataProcRoutineDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QAddDataProcRoutineDialogBox ThisClass;
  typedef QDialog Base;
  typedef c_data_frame_processor_routine::class_factory processor_class_factory;

  QAddDataProcRoutineDialogBox(QWidget * parent = nullptr);

  const processor_class_factory * selectedClassFactory() const;

protected Q_SLOTS:
  void populateClassList();
  void onFilterTextChanged(const QString &);
  void onClassListCurrentItemChanged(QListWidgetItem *, QListWidgetItem *);
  void onOkClicked();

protected:
  const processor_class_factory * selectedClassFactory_ = nullptr;
  QComboBox * filter_ctl = nullptr;
  QListWidget * classlist_ctl = nullptr;
  QLabel * tooltip_ctl = nullptr;
  QPushButton * ok_ctl = nullptr;
  QPushButton * cancel_ctl = nullptr;
};

#endif /* __QAddDataProcRoutineDialogBox_h__ */
