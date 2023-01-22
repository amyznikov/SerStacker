/*
 * QImageStatistics.h
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageStatistics_h__
#define __QImageStatistics_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/UpdateControls.h>
#include <opencv2/opencv.hpp>


class QImageStatisticsDisplay;
class QImageMeasuresSelector;
class QImageMeasuresSelectorDialogBox;
class QImageMeasuresSelectorDialogBox;

class QImageStatisticsDisplay :
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QImageStatisticsDisplay ThisClass;
  typedef QWidget Base;
  typedef std::function<void(const cv::Mat &, const cv::Mat &,
      cv::Scalar outputs[/*num_outputs*/])> measurefunc;

  struct Measure
  {
    QString name;
    QString description;
    QStringList outputs;
    measurefunc func;
    int column;
    bool enabled;

    Measure(const QString & _name,
        const QString & _description,
        const QStringList & _outputs,
        const measurefunc & _func,
        bool _enabled = false) :
        name(_name),
        description(_description),
        outputs(_outputs),
        func(_func),
        enabled(_enabled),
        column(-1)
    {
    }
  };

  QImageStatisticsDisplay(QWidget * parent = nullptr);

  const std::vector<Measure> & measures() const;

  void setMeasure(int index, bool enable);
  bool hasMeasure(int index) const;

  void setImage(cv::InputArray image, cv::InputArray mask);
  void setRoi(const QRect & roi);
  void measure();

  void loadParameters();
  void saveParameters();

protected:
  void setupMeasures();
  void setupToolbar();
  void setupTableView();
  void adjustCurrentRoi(cv::Rect * rc) const;
  void onupdatecontrols() override;
  void onTableViewContextMenuRequested(const QPoint &pos);
  void onTableViewCurrentCellChanged(int currentRow, int currentColumn, int previousRow, int previousColumn);


protected:
  cv::Mat image_;
  cv::Mat mask_;
  QRect roi_;
  std::vector<Measure> measures_;

  QVBoxLayout * lv_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QTableWidget * table_ = nullptr;
  QImageMeasuresSelectorDialogBox * metricsSelector_ctl = nullptr;

  QAction * saveToFileAction_ = nullptr;
  QAction * copyToClipboardAction_ = nullptr;
  QAction * clearTableAction_ = nullptr;
  QAction * selectMeasurementsAction_ = nullptr;
  QAction * incrementalMeasurementsAction_ = nullptr;
  QAction * measureAction_ = nullptr;
};

class QImageMeasuresSelector :
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QImageMeasuresSelector ThisClass;
  typedef QWidget Base;

  QImageMeasuresSelector(QWidget * parent = nullptr);

  void setStatisticsDisplay(QImageStatisticsDisplay * statisticsDisplay);
  QImageStatisticsDisplay * statisticsDisplay() const;

protected:
  void onupdatecontrols() override;
  void onItemChanged(QTreeWidgetItem *item, int column);

protected:

  enum TreeItemType {
    MeasureNameItemType = QTreeWidgetItem::UserType+1,
    MeasureDescriptionType,
  };

  QVBoxLayout * lv_ = nullptr;
  QTreeWidget * treeview_ = nullptr;
  QImageStatisticsDisplay * display_ = nullptr;
};


class QImageMeasuresSelectorDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QImageMeasuresSelectorDialogBox ThisClass;
  typedef QDialog Base;

  QImageMeasuresSelectorDialogBox(QImageStatisticsDisplay * statisticsDisplay);
  QImageMeasuresSelectorDialogBox(const QString & title, QImageStatisticsDisplay * statisticsDisplay);


Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QImageMeasuresSelector * selector_ = nullptr;
};

class QImageStatisticsDisplayDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QImageStatisticsDisplayDialogBox ThisClass;
  typedef QDialog Base;

  QImageStatisticsDisplayDialogBox(QWidget * parent = nullptr);
  QImageStatisticsDisplayDialogBox(const QString & title, QWidget * parent = nullptr);

  void setImage(cv::InputArray image, cv::InputArray mask);
  void setRoi(const QRect & roi);

  QImageStatisticsDisplay * display() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QImageStatisticsDisplay * display_ = nullptr;
};

Q_DECLARE_METATYPE(const QImageStatisticsDisplay::Measure*);

#endif /* __QImageStatistics_h__ */
