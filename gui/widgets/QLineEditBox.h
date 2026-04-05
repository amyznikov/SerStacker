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
    _lineEdit->setText(text);
  }

  QString text() const
  {
    return _lineEdit->text();
  }

  QString displayText() const
  {
    return _lineEdit->displayText();
  }

  QString placeholderText() const
  {
    return _lineEdit->placeholderText();
  }

  void setPlaceholderText(const QString & text)
  {
    return _lineEdit->setPlaceholderText(text);
  }

  QString inputMask() const
  {
    return _lineEdit->inputMask();
  }

  void setInputMask(const QString &inputMask)
  {
    return _lineEdit->setInputMask(inputMask);
  }

  int maxLength() const
  {
    return _lineEdit->maxLength();
  }

  void setMaxLength(int v)
  {
    return _lineEdit->setMaxLength(v);
  }

  void setFrame(bool v)
  {
    return _lineEdit->setFrame(v);
  }

  bool hasFrame() const
  {
    return _lineEdit->hasFrame();
  }

  void setClearButtonEnabled(bool enable)
  {
    return _lineEdit->setClearButtonEnabled(enable);
  }

  bool isClearButtonEnabled() const
  {
    return _lineEdit->isClearButtonEnabled();
  }

  QLineEdit::EchoMode echoMode() const
  {
    return _lineEdit->echoMode();
  }

  void setEchoMode(QLineEdit::EchoMode v)
  {
    return _lineEdit->setEchoMode(v);
  }

  bool isReadOnly() const
  {
    return _lineEdit->isReadOnly();
  }

  void setReadOnly(bool v)
  {
    return _lineEdit->setReadOnly(v);
  }

  bool isModified() const
  {
    return _lineEdit->isModified();
  }

  void setModified(bool v)
  {
    return _lineEdit->setModified(v);
  }

  QHBoxLayout * layout() const
  {
    return _layout;
  }

  QLineEdit * lineEdit() const
  {
    return _lineEdit;
  }

Q_SIGNALS:
  void textChanged();
  void returnPressed();

protected:
  QHBoxLayout * _layout = nullptr;
  QLineEdit * _lineEdit = nullptr;
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


class QMultiLineEditBox :
    public QPlainTextEdit
{
  Q_OBJECT;
public:
  typedef QMultiLineEditBox ThisClass;
  typedef QPlainTextEdit Base;

  explicit QMultiLineEditBox(QWidget * parent = nullptr) :
      ThisClass("", parent)
  {
  }

  explicit QMultiLineEditBox(const QString & text, QWidget * parent = nullptr) :
      Base(text, parent)
  {
    document()->setModified(false);
    setToolTip("Press Ctrl+Enter to apply");
  }

  void setPlainText(const QString & text)
  {
    Base::setPlainText(text);
    document()->setModified(false);
  }

Q_SIGNALS:
  void editFinished();

protected:
  void keyPressEvent(QKeyEvent * e) override
  {
    const int key = e->key();
    if( ( key == Qt::Key_Return || key == Qt::Key_Enter) && (e->modifiers() & Qt::ControlModifier) ) {
      Q_EMIT editFinished();
    }
    else {
      Base::keyPressEvent(e);
    }
  }

  void focusOutEvent(QFocusEvent * e) override
  {
    if( document()->isModified() ) {
      Q_EMIT editFinished();
      document()->setModified(false);
    }
    Base::focusOutEvent(e);
  }

protected:
};

#endif /* __QLineEditBox_h__ */
