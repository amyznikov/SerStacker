/*
 * QThumbnailsQuickFilterOptions.h
 *
 *  Created on: Jul 6, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QThumbnailsQuickFilterOptions_h__
#define __QThumbnailsQuickFilterOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/addctrl.h>

//
//const extern struct QtMatchingFlags_desc {
//  const char * name;
//  Qt::MatchFlags value;
//} QtMatchingFlags[];
//

QString toQString(Qt::MatchFlags m);

Qt::MatchFlags fromQString(const QString & s,
    Qt::MatchFlags defval );

bool matchQuickFilter(const QString & text,
    const QString & pattern,
    Qt::MatchFlags flags);


typedef QEnumComboBox<Qt::MatchFlag> QThumbnailsQuickFilterMatchingFlagsCombo;

//class QThumbnailsQuickFilterMatchingFlagsCombo
//    : public QEnumComboBox<Qt::MatchFlag>
//{
//  Q_OBJECT;
//public:
//  typedef QEnumComboBox<Qt::MatchFlag> Base;
//
//  QThumbnailsQuickFilterMatchingFlagsCombo(QWidget * parent = Q_NULLPTR)
//      : Base(parent)
//    {}
//
//};
//

class QThumbnailsQuickFilterOptions
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QThumbnailsQuickFilterOptions ThisClass;
  typedef QSettingsWidget Base;

  QThumbnailsQuickFilterOptions(QWidget * parent = Q_NULLPTR);
  ~QThumbnailsQuickFilterOptions();

  void setSearchText(const QString & v);
  QString searchText() const;

  void setMatchingFlags(Qt::MatchFlags v);
  Qt::MatchFlags matchingFlags() const;

  void setInvertMatch(bool v);
  bool invertMatch() const;

protected:
  void loadSavedFilters();
  void saveFilters();

protected slots:
  void onSearchTextChanged(const QString & );
protected:
  QComboBox * searchText_ctl = Q_NULLPTR;
  QThumbnailsQuickFilterMatchingFlagsCombo * matchingFlags_ctl = Q_NULLPTR;
  QCheckBox * caseSensitivity_ctl = Q_NULLPTR;
  QCheckBox * invertMatch_ctl = Q_NULLPTR;
};


class QThumbnailsQuickFilterDialogBox
    : public QDialog
{
  Q_OBJECT;
public:
  typedef QThumbnailsQuickFilterDialogBox ThisClass;
  typedef QDialog Base;

  QThumbnailsQuickFilterDialogBox(QWidget * parent = Q_NULLPTR);

public:
  void setSearchText(const QString & v);
  QString searchText() const;

  void setMatchingFlags(Qt::MatchFlags v);
  Qt::MatchFlags matchingFlags() const;

  void setInvertMatch(bool v);
  bool invertMatch() const;

signals:
  void parameterChanged();

protected:
  QThumbnailsQuickFilterOptions * form_ = Q_NULLPTR;
};


#endif /* __QThumbnailsQuickFilterOptions_h__ */
