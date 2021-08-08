/*
 * QImageProcessorCollection.h
 *
 *  Created on: Aug 8, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessorCollection_h__
#define __QImageProcessorCollection_h__

#include <QtCore/QtCore>
#include <core/improc/c_image_processor.h>


class QImageProcessorsCollection
    : public QObject
{
  Q_OBJECT;
public:
  typedef QImageProcessorsCollection ThisClass;
  typedef QObject Base;

  static bool load();
  static bool save();
  static int size();
  static bool empty();
  static const c_image_processor::ptr & item(int pos);
  static void add(const c_image_processor::ptr & p, bool emit_notify = true);
  static bool insert(int pos, const c_image_processor::ptr & p, bool emit_notify = true);
  static bool remove(const c_image_processor::ptr & p, bool emit_notify = true);
  static bool remove_at(int pos, bool emit_notify = true);
  static int indexof(const c_image_processor::ptr & p);
  static int indexof(const std::string & name);
  static int indexof(const QString & name);

  // for QObject::connect()
  static QImageProcessorsCollection * instance();

signals:
  void collectionChanged();

protected:
  QImageProcessorsCollection();
  virtual ~QImageProcessorsCollection();

protected:
  static QImageProcessorsCollection instance_;
  static c_image_processor_collection::ptr processors_;
};



#endif /* __QImageProcessorCollection_h__ */
