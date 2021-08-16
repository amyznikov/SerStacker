/*
 * QImageProcessorCollection.cc
 *
 *  Created on: Aug 8, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorsCollection.h"
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

QImageProcessorsCollection QImageProcessorsCollection::instance_;

c_image_processor_collection::ptr QImageProcessorsCollection::processors_ =
    c_image_processor_collection::create();

QImageProcessorsCollection::QImageProcessorsCollection()
{
}

QImageProcessorsCollection::~QImageProcessorsCollection()
{
}

// for QObject::connect()
QImageProcessorsCollection * QImageProcessorsCollection::instance()
{
  return &instance_;
}

bool QImageProcessorsCollection::load()
{
  processors_->clear();
  processors_->load(c_image_processor_collection::default_processor_collection_path());
  if ( processors_->empty() ) {
    processors_->emplace_back(c_image_processor::create("Default"));
  }
  emit instance()->collectionChanged();
  return true;
}

bool QImageProcessorsCollection::save()
{
  return processors_->save();
}

int QImageProcessorsCollection::size()
{
  return processors_->size();
}

bool QImageProcessorsCollection::empty()
{
  return processors_->empty();
}

const c_image_processor::ptr & QImageProcessorsCollection::item(int pos)
{
  return processors_->at(pos);
}



void QImageProcessorsCollection::add(const c_image_processor::ptr & p, bool emit_notify)
{
  processors_->emplace_back(p);

  if ( emit_notify ) {
    emit instance_.collectionChanged();
  }
}

bool QImageProcessorsCollection::insert(int pos, const c_image_processor::ptr & p, bool emit_notify)
{
  if( pos < 0 || pos >= size() ) {
    CF_FATAL("POSSIBLE APP BUG: attempt to insert into position %d when array size=%d", pos, size());
    return false;
  }

  processors_->insert(processors_->begin() + pos, p);

  if ( emit_notify ) {
    emit instance_.collectionChanged();
  }

  return true;
}

bool QImageProcessorsCollection::remove(const c_image_processor::ptr & p, bool emit_notify)
{
  c_image_processor_collection::iterator pos =
      processors_->find(p);

  if( pos != processors_->end() ) {
    processors_->erase(pos);

    if ( emit_notify ) {
      emit instance_.collectionChanged();
    }

    return true;
  }

  return false;
}

bool QImageProcessorsCollection::remove_at(int pos, bool emit_notify)
{
  if( pos < 0 || pos >= size() ) {
    CF_FATAL("POSSIBLE APP BUG: attempt to remove from position %d when array size=%d", pos, size());
    return false;
  }

  processors_->erase(processors_->begin() + pos);
  if ( emit_notify ) {
    emit instance_.collectionChanged();
  }

  return true;
}

int QImageProcessorsCollection::indexof(const c_image_processor::ptr & p)
{
  c_image_processor_collection::iterator pos =
      processors_->find(p);

  return pos != processors_->end() ?
      pos - processors_->begin() : -1;
}

int QImageProcessorsCollection::indexof(const std::string & name)
{
  c_image_processor_collection::iterator pos =
      processors_->find(name);

  return pos != processors_->end() ?
      pos - processors_->begin() : -1;

}

int QImageProcessorsCollection::indexof(const QString & name)
{
  c_image_processor_collection::iterator pos =
      processors_->find(name.toStdString());

  return pos != processors_->end() ?
      pos - processors_->begin() : -1;

}

///////////////////////////////////////////////////////////////////////////////

QImageProcessorSelectionCombo::QImageProcessorSelectionCombo(QWidget * parent) :
    Base(parent)
{
  setEditable(false);
  setMinimumContentsLength(12);
  setSizeAdjustPolicy(QComboBox::AdjustToContents);


  connect(QImageProcessorsCollection::instance(), &QImageProcessorsCollection::collectionChanged,
      this, &ThisClass::refresh);

  refresh();
}

c_image_processor::ptr QImageProcessorSelectionCombo::processor(int index) const
{
  if ( index < 1 || (index = QImageProcessorsCollection::indexof(currentText()) ) < 0 ) {
    return nullptr;
  }

  return QImageProcessorsCollection::item(index);
}

bool QImageProcessorSelectionCombo::setCurrentProcessor(const c_image_processor::ptr & processor)
{
  int current_index = 0; // "None" in refresh()
  if ( processor && (current_index = findText(processor->cname())) < 0) {
    current_index = 0;
  }

  setCurrentIndex(current_index);

  return currentIndex() > 0;
}

c_image_processor::ptr QImageProcessorSelectionCombo::currentProcessor() const
{
  return processor(currentIndex());
}

void QImageProcessorSelectionCombo::refresh()
{
  const bool wasEnabled =
      isEnabled();

  if ( wasEnabled ) {
    setEnabled(false);
  }

  const QString current_processor_name =
      currentText();

  clear();
  addItem("None");

  for ( int i = 0, n = QImageProcessorsCollection::size(); i < n; ++i ) {

    const c_image_processor::ptr processor =
        QImageProcessorsCollection::item(i);

    if( processor ) {
      addItem(processor->cname(), processor->cfilename());
    }
  }

  if ( !current_processor_name.isEmpty() ) {
    const int index =
        findText(current_processor_name);

    if ( index >= 0 ) {
      setCurrentIndex(index);
    }
  }

  if ( wasEnabled ) {
    setEnabled(true);
  }

  if ( currentText() != current_processor_name ) {
    emit currentIndexChanged(currentIndex());
  }
}


///////////////////////////////////////////////////////////////////////////////
