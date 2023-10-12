/*
 * QMatrixEdit.cc
 *
 *  Created on: Aug 2, 2023
 *      Author: amyznikov
 */

#include "QMatrixEdit.h"
#include <gui/widgets/qsprintf.h>
#include <core/ssprintf.h>
#include <core/debug.h>
#include "QCameraMatrixDialogBox.h"


QMatrixEdit::QMatrixEdit(QWidget * parent) :
  ThisClass(3,3, parent)
{
}

QMatrixEdit::QMatrixEdit(int rows, int cols, QWidget * parent) :
    Base(parent)
{
  setContentsMargins(0,0,0,0);
  setSizeAdjustPolicy(AdjustToContents);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

  setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  //setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  // setEditTriggers(QAbstractItemView::NoEditTriggers);

  horizontalHeader()->hide();
  verticalHeader()->hide();

  setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

  setMatrixSize(rows, cols);

  connect(this, &QTableWidget::customContextMenuRequested,
      this, &ThisClass::onTableViewContextMenuRequested);

  connect(this, &QTableWidget::itemChanged,
      [this](QTableWidgetItem * item) {
        if ( enable_signals_ ) {
          Q_EMIT matrixChanged();
        }
      });

}

void QMatrixEdit::setMatrixSize(int rows, int cols)
{
  enable_signals_ = false;
  Base::setRowCount(rows);
  Base::setColumnCount(cols);
  Base::resizeRowsToContents();
  enable_signals_ = true;
}

int QMatrixEdit::rows() const
{
  return rowCount();
}

int QMatrixEdit::cols() const
{
  return columnCount();
}

QTableWidgetItem* QMatrixEdit::setItem(int r, int c, const QString & text)
{
  QTableWidgetItem *item =
      Base::item(r, c);

  if( item ) {
    item->setText(text);
  }
  else {
    Base::setItem(r, c,
        item = new QTableWidgetItem(text));
  }

  return item;
}

void QMatrixEdit::onTableViewContextMenuRequested(const QPoint &pos)
{
  QMenu menu;

  menu.addAction("Copy Matrix",
      [this]() {
        QApplication::clipboard()->setText(getMatrixText(false));
      });

  menu.addAction("Copy Matrix (Transposed)",
      [this]() {
        QApplication::clipboard()->setText(getMatrixText(true));
      });

  QString text =
      QApplication::clipboard()->text();

  if( !text.isEmpty() ) {

    menu.addSeparator();

    menu.addAction("Paste",
        [this, text]() {
          setMatrixText(text, false);
        });

    menu.addAction("Paste (Transposed)",
        [this, text]() {
          setMatrixText(text, true);
        });
  }

  if ( !menu.isEmpty() ) {
    menu.addSeparator();
  }


//  menu.addAction("Saved Cameras...",
//      [this]() {
//
//        // QCameraMatrixDialogBox dlgbox(this);
//        // if ( dlgbox.exec() == QDialog::Accepted ) {
//        //  }
//
//      });
//


  if ( !menu.isEmpty() ) {
    menu.exec(viewport()->mapToGlobal(pos));
  }

}


void QMatrixEdit::setMatrix(const cv::Mat & m)
{
  enable_signals_ = false;

  const int nr = rows();
  const int nc = cols();

  if( m.rows != nr || m.cols != nc || m.channels() != 1 ) {
    CF_ERROR("Invalid matrix specified: %dx%d %d channels, must be %dx%d single channel",
        m.rows, m.cols, m.channels(), nr, nc);
    return;
  }

  cv::Mat1d mm;

  if( m.depth() == mm.depth() ) {
    mm = m;
  }
  else {
    m.convertTo(mm, mm.depth());
  }

  for( int r = 0; r < nr; ++r ) {
    for( int c = 0; c < nc; ++c ) {
      setItem(r, c, qsprintf("%+g", mm[r][c]));
    }
  }

  enable_signals_ = true;

  Q_EMIT matrixChanged();
}

cv::Mat QMatrixEdit::matrix() const
{
  const int nr = rows();
  const int nc = cols();

  cv::Mat1d m = cv::Mat1d::zeros(nr, nc);

  for( int r = 0; r < nr; ++r ) {
    for( int c = 0; c < nc; ++c ) {

      const QTableWidgetItem *item =
          Base::item(r, c);

      double v = 0;

      if( item && sscanf(item->text().toUtf8().constData(), "%lf", &v) == 1 ) {
        m[r][c] = v;
      }
    }
  }

  return m;
}

void QMatrixEdit::setMatrixText(const QString & text, bool transposed)
{
  enable_signals_ = false;

  const std::vector<std::string> tokens =
      strsplit(text.toStdString(),
          " \t\n\r\b;,[](){}`\"@#'%^&*<>?|\\/!~");


  const int nr = rows();
  const int nc = cols();

  int i = 0;

  for( int r = 0; r < nr; ++r ) {
    for( int c = 0; c < nc; ++c ) {
      setItem(r, c, "");
    }
  }

  if ( !transposed  ) {

    for( int r = 0; r < nr && i < tokens.size(); ++r ) {
      for( int c = 0; c < nc && i < tokens.size(); ++c ) {
        setItem(r, c, tokens[i++].c_str());
      }
    }
  }
  else {

    for( int c = 0; c < nc && i < tokens.size(); ++c ) {
      for( int r = 0; r < nr && i < tokens.size(); ++r ) {
        setItem(r, c, tokens[i++].c_str());
      }
    }
  }

  enable_signals_ = true;

  Q_EMIT matrixChanged();
}

QString QMatrixEdit::getMatrixText(bool transposed) const
{
  const int nr = rows();
  const int nc = cols();

  QString text;

  if ( !transposed  ) {

    for( int r = 0; r < nr; ++r ) {
      for( int c = 0; c < nc; ++c ) {

        QTableWidgetItem *item =
            Base::item(r, c);

        if( item ) {
          text.append(item->text());
        }
        else {
          text.append("0");
        }

        text.append(c == nc -1 ? "\n" : "\t");
      }
    }
  }
  else {
    for( int c = 0; c < nc; ++c ) {
      for( int r = 0; r < nr; ++r ) {

        QTableWidgetItem *item =
            Base::item(r, c);

        if( item ) {
          text.append(item->text());
        }
        else {
          text.append("0");
        }

        text.append(c == nc - 1 ? "\n" : "\t");
      }
    }
  }

  return text;
}

