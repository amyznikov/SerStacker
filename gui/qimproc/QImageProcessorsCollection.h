/*
 * QImageProcessorCollection.h
 *
 *  Created on: Aug 8, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessorCollection_h__
#define __QImageProcessorCollection_h__

#include <QtWidgets/QtWidgets>
#include <core/improc/c_image_processor.h>


class QImageProcessorsCollection :
    public QObject
{
  Q_OBJECT;
public:
  typedef QImageProcessorsCollection ThisClass;
  typedef QObject Base;

  static bool load();
  static bool save();
  static int size();
  static bool empty();
  static const c_image_processor::sptr & item(int pos);
  static void add(const c_image_processor::sptr & p, bool emit_notify = true);
  static bool insert(int pos, const c_image_processor::sptr & p, bool emit_notify = true);
  static bool remove(const c_image_processor::sptr & p, bool emit_notify = true);
  static bool remove_at(int pos, bool emit_notify = true);
  static int indexof(const c_image_processor::sptr & p);
  static int indexof(const std::string & name);
  static int indexof(const QString & name);

  // for QObject::connect()
  static QImageProcessorsCollection * instance();

Q_SIGNALS:
  void collectionChanged();

protected:
  QImageProcessorsCollection();
  virtual ~QImageProcessorsCollection();

protected:
  static QImageProcessorsCollection _instance;
};

class QImageProcessorSelectionCombo :
    public QComboBox
{
  Q_OBJECT;
public:
  typedef QImageProcessorSelectionCombo ThisClass;
  typedef QComboBox Base;

  QImageProcessorSelectionCombo(QWidget * parent = nullptr);

  bool setCurrentProcessor(const c_image_processor::sptr & processor);
  c_image_processor::sptr currentProcessor() const;
  c_image_processor::sptr processor(int index) const;

public slots:
  void refresh();
};

class QImageProcessorSelectionCombo2 :
    public QComboBox
{
  Q_OBJECT;
public:
  typedef QImageProcessorSelectionCombo2 ThisClass;
  typedef QComboBox Base;

  QImageProcessorSelectionCombo2(QWidget * parent = nullptr);

  bool setCurrentProcessor(const QString & name);
  QString currentProcessor() const;
  QString processor(int index) const;

public slots:
  void refresh();
};


#endif /* __QImageProcessorCollection_h__ */
