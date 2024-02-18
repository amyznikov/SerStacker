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

class QLineEditBox :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QLineEditBox ThisClass;
  typedef QWidget Base;

  explicit QLineEditBox(QWidget *parent = nullptr);
  explicit QLineEditBox(const QString &, QWidget *parent = nullptr);

  template<class T>
  void setValue(const T & v)
  {
    setText(toQString(v));
  }

  void setText(const QString & text)
  {
    lineEdit_->setText(text);
  }

  QString text() const
  {
    return lineEdit_->text();
  }

  QString displayText() const
  {
    return lineEdit_->displayText();
  }

  QString placeholderText() const
  {
    return lineEdit_->placeholderText();
  }

  void setPlaceholderText(const QString & text)
  {
    return lineEdit_->setPlaceholderText(text);
  }

  QString inputMask() const
  {
    return lineEdit_->inputMask();
  }

  void setInputMask(const QString &inputMask)
  {
    return lineEdit_->setInputMask(inputMask);
  }

  int maxLength() const
  {
    return lineEdit_->maxLength();
  }

  void setMaxLength(int v)
  {
    return lineEdit_->setMaxLength(v);
  }

  void setFrame(bool v)
  {
    return lineEdit_->setFrame(v);
  }

  bool hasFrame() const
  {
    return lineEdit_->hasFrame();
  }

  void setClearButtonEnabled(bool enable)
  {
    return lineEdit_->setClearButtonEnabled(enable);
  }

  bool isClearButtonEnabled() const
  {
    return lineEdit_->isClearButtonEnabled();
  }

  QLineEdit::EchoMode echoMode() const
  {
    return lineEdit_->echoMode();
  }

  void setEchoMode(QLineEdit::EchoMode v)
  {
    return lineEdit_->setEchoMode(v);
  }

  bool isReadOnly() const
  {
    return lineEdit_->isReadOnly();
  }

  void setReadOnly(bool v)
  {
    return lineEdit_->setReadOnly(v);
  }

  bool isModified() const
  {
    return lineEdit_->isModified();
  }

  void setModified(bool v)
  {
    return lineEdit_->setModified(v);
  }

  QHBoxLayout * layout() const
  {
    return layout_;
  }

  QLineEdit * lineEdit() const
  {
    return lineEdit_;
  }

Q_SIGNALS:
  void textChanged();
  void returnPressed();

protected:
  QHBoxLayout * layout_ = nullptr;
  QLineEdit * lineEdit_ = nullptr;
};

class QNumericBox :
    public QLineEditBox
{
  Q_OBJECT;
public:
  typedef QNumericBox ThisClass;
  typedef QLineEditBox Base;

  explicit QNumericBox(QWidget *parent = nullptr) : Base(parent)
  {
    // lineEdit_->setMaximumWidth(NUMERICAL_FIELD_DEFAULT_MAX_WIDTH);
  }

  explicit QNumericBox(const QString & s, QWidget * parent = nullptr) : Base(s, parent)
  {
    //lineEdit_->setMaximumWidth(NUMERICAL_FIELD_DEFAULT_MAX_WIDTH);
  }
};

#endif /* __QLineEditBox_h__ */
