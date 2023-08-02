/*
 * QMatrixEdit.h
 *
 *  Created on: Aug 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMatrixEdit_h__
#define __QMatrixEdit_h__

#include <QtWidgets/QtWidgets>
#include <opencv2/opencv.hpp>

class QMatrixEdit :
    public QTableWidget
{
  Q_OBJECT;
public:
  typedef QMatrixEdit ThisClass;
  typedef QTableWidget Base;

  QMatrixEdit(QWidget * parent = nullptr);
  QMatrixEdit(int rows, int cols, QWidget * parent = nullptr);

  void setMatrixSize(int rows, int cols);

  int rows() const;
  int cols() const;

  void setMatrix(const cv::Mat & m);
  cv::Mat matrix() const;

  void setMatrixText(const QString & text, bool transposed = false);
  QString getMatrixText(bool transposed = false) const;


  template<class T, int r, int c>
  bool getMatrix(cv::Matx<T, r, c> * mx)
  {
    cv::Mat m = matrix();
    if( m.rows == r && m.cols == c && m.channels() == 1 ) {

      if( m.depth() != cv::DataType<T>::depth ) {
        m.convertTo(m, cv::DataType<T>::depth);
      }

      *mx = cv::Matx<T, r, c>(m.ptr<const T>());
      return true;
    }

    return false;
  }

Q_SIGNALS:
  void matrixChanged();

protected:
  QTableWidgetItem *setItem(int r, int c, const QString & text);
  void onTableViewContextMenuRequested(const QPoint &pos);

protected:
  bool enable_signals_ = true;
};

#endif /* __QMatrixEdit_h__ */
