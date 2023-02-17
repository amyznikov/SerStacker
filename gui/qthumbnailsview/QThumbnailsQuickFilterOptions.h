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

QString toQString(Qt::MatchFlags m);

Qt::MatchFlags fromQString(const QString & s,
    Qt::MatchFlags defval );

bool matchQuickFilter(const QString & text,
    const QString & pattern,
    Qt::MatchFlags flags);


typedef QEnumComboBox<Qt::MatchFlag> QThumbnailsQuickFilterMatchingFlagsCombo;

class QThumbnailsQuickFilterOptions
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QThumbnailsQuickFilterOptions ThisClass;
  typedef QSettingsWidget Base;

  QThumbnailsQuickFilterOptions(QWidget * parent = nullptr);
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

protected Q_SLOTS:
  void onSearchTextCurrentIndexChanged(int);
  void onSearchTextChanged(const QString & );
protected:
  QComboBox * searchText_ctl = nullptr;
  QThumbnailsQuickFilterMatchingFlagsCombo * matchingFlags_ctl = nullptr;
  QCheckBox * caseSensitivity_ctl = nullptr;
  QCheckBox * invertMatch_ctl = nullptr;
};


class QThumbnailsQuickFilterDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QThumbnailsQuickFilterDialogBox ThisClass;
  typedef QDialog Base;

  QThumbnailsQuickFilterDialogBox(QWidget * parent = nullptr);

public:
  void setSearchText(const QString & v);
  QString searchText() const;

  void setMatchingFlags(Qt::MatchFlags v);
  Qt::MatchFlags matchingFlags() const;

  void setInvertMatch(bool v);
  bool invertMatch() const;

Q_SIGNALS:
  void parameterChanged();

protected:
  QThumbnailsQuickFilterOptions * form_ = nullptr;
};


#endif /* __QThumbnailsQuickFilterOptions_h__ */
