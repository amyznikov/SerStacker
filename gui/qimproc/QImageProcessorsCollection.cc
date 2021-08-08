/*
 * QImageProcessorCollection.cc
 *
 *  Created on: Aug 8, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorsCollection.h"

#include <core/debug.h>

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
  return processors_->load(c_image_processor_collection::default_processor_collection_path());
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

