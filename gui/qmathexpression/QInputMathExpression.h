/*
 * QInputMathExpression.h
 *
 *  Created on: Oct 25, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputMathExpressionWidget_h__
#define __QInputMathExpressionWidget_h__

#include <QtWidgets/QtWidgets>

class QMathExpressionTextEdit :
    public QPlainTextEdit
{
  Q_OBJECT;
public:
  typedef QMathExpressionTextEdit ThisClass;
  typedef QPlainTextEdit Base;

  QMathExpressionTextEdit(QWidget * parent = nullptr);

Q_SIGNALS:
  void editFinished();

protected:
  // void focusInEvent(QFocusEvent *e) override;
  void focusOutEvent(QFocusEvent *e) override;
  void keyPressEvent(QKeyEvent *e) override;

};



class QInputMathExpressionWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QInputMathExpressionWidget ThisClass;
  typedef QWidget Base;

  QInputMathExpressionWidget(QWidget * parent = nullptr);

  void setText(const QString & text);
  QString text() const;

  void setHelpString(const QString & s);
  const QString & helpString() const;

  void showHelpString();

Q_SIGNALS:
  void apply();

protected:
  QMathExpressionTextEdit * expressionTextBox_ctl = nullptr;
  QPushButton * apply_ctl = nullptr;
  QPushButton * showFunctions_ctl = nullptr;
  QString _helpString;
};

class QInputMathExpressionDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QInputMathExpressionDialogBox ThisClass;
  typedef QDialog Base;

  QInputMathExpressionDialogBox(QWidget * parent = nullptr);

  void setText(const QString & text);
  QString text() const;

protected:
  QInputMathExpressionWidget * _controlWidget = nullptr;
};

#endif /* __QInputMathExpressionWidget_h__ */
