/*
 * QAddRoutineDialog.cc
 *
 *  Created on: Aug 7, 2021
 *      Author: amyznikov
 */

#include "QAddRoutineDialog.h"


QAddRoutineDialog::QAddRoutineDialog(QWidget * parent)
  : Base(parent)
{
  setWindowTitle("Select processing routine");

  QVBoxLayout * vbox1 = new QVBoxLayout(this);
  QHBoxLayout * hbox1 = new QHBoxLayout();
  vbox1->setAlignment(Qt::AlignTop);

  QHBoxLayout * hbox2 = new QHBoxLayout();
  QVBoxLayout * vbox2 = new QVBoxLayout();


  vbox2->addWidget(filter_ctl = new QComboBox(this));
  vbox2->addWidget(classlist_ctl = new QListWidget(this));

  hbox1->addLayout(vbox2);
  hbox1->addWidget(tooltip_ctl = new QLabel(this));
  tooltip_ctl->setTextFormat(Qt::TextFormat::RichText);
  tooltip_ctl->setWordWrap(true);

  hbox2->addWidget(ok_ctl = new QPushButton("OK", this));
  hbox2->addWidget(cancel_ctl = new QPushButton("Canel", this));

  vbox1->addLayout(hbox1, 1000);
  vbox1->addLayout(hbox2, 1);

  populateClassList();

  filter_ctl->setEditable(true);
  filter_ctl->setAutoCompletion(true);
  filter_ctl->setMinimumContentsLength(32);
  filter_ctl->setInsertPolicy(QComboBox::InsertAtTop);

  ok_ctl->setDefault(true);
  ok_ctl->setAutoDefault(true);

  connect(filter_ctl, &QComboBox::currentTextChanged,
      this, &ThisClass::onFilterTextChanged);

  connect(classlist_ctl, &QListWidget::currentItemChanged,
      this, &ThisClass::onClassListCurrentItemChanged);

  connect(ok_ctl, &QPushButton::clicked,
      this, &ThisClass::onOkClicked);

  connect(cancel_ctl, &QPushButton::clicked,
      this, &QDialog::reject);
}

void QAddRoutineDialog::populateClassList()
{
  classlist_ctl->clear();
  selectedClassFactory_ = Q_NULLPTR;

  for ( const c_image_processor_routine::class_factory * c : c_image_processor_routine::class_list() ) {
    QListWidgetItem * item = new QListWidgetItem(c->class_name.c_str(), classlist_ctl);
    item->setData(Qt::UserRole, QVariant::fromValue((void*)c));
  }

}

void QAddRoutineDialog::onFilterTextChanged(const QString & filter)
{
  if ( filter.isEmpty() ) {
    for ( int  i = 0, n = classlist_ctl->count(); i < n; ++i ) {
      classlist_ctl->item(i)->setHidden(false);
    }
  }
  else {
    for ( int i = 0, n = classlist_ctl->count(); i < n; ++i ) {
      QListWidgetItem * item = classlist_ctl->item(i);
      item->setHidden(!item->text().contains(filter, Qt::CaseInsensitive));
    }
  }
}

void QAddRoutineDialog::onClassListCurrentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
  if ( !current ) {
    tooltip_ctl->clear();
  }
  else {
    const c_image_processor_routine::class_factory * c =
        static_cast<const c_image_processor_routine::class_factory *>(
        current->data(Qt::UserRole).value<void*>());

    if ( c ) {
      tooltip_ctl->setText(c->tooltip.c_str());
    }
    else {
      tooltip_ctl->setText("cast fails");
    }
  }
}

void QAddRoutineDialog::onOkClicked()
{
  QListWidgetItem * current_item = classlist_ctl->currentItem();
  if ( current_item ) {
    selectedClassFactory_ = static_cast<const c_image_processor_routine::class_factory *>(
        current_item->data(Qt::UserRole).value<void*>());
    accept();
  }
}

const c_image_processor_routine::class_factory * QAddRoutineDialog::selectedClassFactory() const
{
  return selectedClassFactory_;
}
