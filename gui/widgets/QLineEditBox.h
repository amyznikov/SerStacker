/*
 * QLineEditBox.h
 *
 *  Created on: Oct 7, 2019
 *      Author: amyznikov
 */

#ifndef __QLineEditBox_h__
#define __QLineEditBox_h__

#include "settings.h"

///////////////////////////////////////////////////////////////////////////////

#define NUMERICAL_FIELD_DEFAULT_MAX_WIDTH  128

class QLineEditBox
  : public QLineEdit
{
  Q_OBJECT;
public:
  typedef QLineEditBox ThisClass;
  typedef QLineEdit Base;
  explicit QLineEditBox(QWidget *parent = Q_NULLPTR);
  explicit QLineEditBox(const QString &, QWidget *parent = Q_NULLPTR);

  template<class T>
  void setValue(const T & v) {
    setText(toQString(v));
  }

signals:
  void textChanged();

protected:
  void focusInEvent(QFocusEvent* e) override;
private:
  void construct(void);
  QString previousText;
};

class QNumberEditBox
  : public QLineEditBox
{
  Q_OBJECT;
public:
  typedef QLineEditBox Base;
  explicit QNumberEditBox(QWidget *parent = Q_NULLPTR)
    : Base(parent) {
      setMaximumWidth(NUMERICAL_FIELD_DEFAULT_MAX_WIDTH);
    }

  explicit QNumberEditBox(const QString & s, QWidget *parent = Q_NULLPTR)
    : Base(s, parent) {
      setMaximumWidth(NUMERICAL_FIELD_DEFAULT_MAX_WIDTH);
    }
};

#endif /* __QLineEditBox_h__ */
