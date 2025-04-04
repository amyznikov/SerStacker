/*
 * QThumbnailsQuickFilterOptions.cc
 *
 *  Created on: Jul 6, 2021
 *      Author: amyznikov
 */

#include "QThumbnailsQuickFilterOptions.h"
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
//# include <QtCore5Compat/QRegExp>
#endif

#include <core/debug.h>


#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
# define MatchRegularExpression MatchRegExp
#endif


template<>
const c_enum_member * members_of<Qt::MatchFlag>()
{
  static const c_enum_member members[] = {
      {Qt::MatchWildcard, "Wildcard", },
      {Qt::MatchContains , "Contains", },
      {Qt::MatchStartsWith, "StartsWith", },
      {Qt::MatchEndsWith, "EndsWith", },
      {Qt::MatchExactly, "Exact", },
      {Qt::MatchRegularExpression, "RegExp", },
      {(Qt::MatchFlag)(0) }
  };
  return members;
}


QString toQString(Qt::MatchFlags v)
{
  v = v & 0xF;
#if !(QT_DEPRECATED_SINCE(5, 15))
  if ( v == Qt::MatchRegExp ) {
    v = Qt::MatchRegularExpression;
  }
  else
#endif
    if ( v == Qt::MatchFixedString ) {
    v = Qt::MatchExactly;
  }

  const c_enum_member * QtMatchingFlags =
      members_of<Qt::MatchFlag>();

  if ( QtMatchingFlags ) {
    for ( uint i = 0; !QtMatchingFlags[i].name.c_str(); ++i ) {
      if ( QtMatchingFlags[i].value == v ) {
        return QtMatchingFlags[i].name.c_str();
      }
    }
  }
  return "";
}

Qt::MatchFlags fromQString(const QString & s, Qt::MatchFlags defval)
{
  const QByteArray a = s.toUtf8();
  const char * cstr = a.data();

  const c_enum_member * QtMatchingFlags =
      members_of<Qt::MatchFlag>();
  if ( QtMatchingFlags ) {
    for ( uint i = 0; !QtMatchingFlags[i].name.c_str(); ++i ) {
      if ( strcasecmp(QtMatchingFlags[i].name.c_str(), cstr) == 0 ) {
        return (Qt::MatchFlags)QtMatchingFlags[i].value;
      }
    }
  }
  return defval;
}


bool matchQuickFilter(const QString & text,
    const QString & pattern,
    Qt::MatchFlags flags)
{
  bool matchStatus = false;

   if ( pattern.isEmpty() ) {
     matchStatus = true;
   }
   else {

     const uint matchType =
         flags & 0x0F;

     const Qt::CaseSensitivity cs =
         (flags & Qt::MatchCaseSensitive) ?
             Qt::CaseSensitive :
             Qt::CaseInsensitive;

     switch ( matchType ) {

     case Qt::MatchFixedString :
       case Qt::MatchExactly : {
       matchStatus = pattern.compare(text, cs) == 0;
       break;
     }

#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
     case Qt::MatchRegExp : {
       matchStatus = QRegExp(pattern, cs, QRegExp::RegExp).exactMatch(text);
       break;
     }
#endif

     case Qt::MatchWildcard : {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
       matchStatus = QRegularExpression::fromWildcard(QStringView(pattern), cs).match(text).hasMatch();
#else
       matchStatus = QRegExp(pattern, cs, QRegExp::Wildcard).exactMatch(text);
#endif

       break;
     }

     case Qt::MatchStartsWith : {
       matchStatus = text.startsWith(pattern, cs);
       break;
     }

     case Qt::MatchEndsWith : {
       matchStatus = text.endsWith(pattern, cs);
       break;
     }
     case Qt::MatchContains :
       default : {
       matchStatus = text.contains(pattern, cs);
       break;
     }
     }

   }

   return matchStatus;

}


QThumbnailsQuickFilterOptions::QThumbnailsQuickFilterOptions(QWidget * parent) :
    Base("QThumbnailsQuickFilterOptions", parent)
{

  searchText_ctl = new QComboBox(this);
  searchText_ctl->setEditable(true);

#if QT_VERSION < QT_VERSION_CHECK(5, 13, 0)
  searchText_ctl->setAutoCompletion(true);
#else
  QCompleter *completer = new QCompleter(this);
  completer->setModel(searchText_ctl->model());
  searchText_ctl->setCompleter(completer);
#endif

  searchText_ctl->setMinimumContentsLength(32);
  searchText_ctl->setInsertPolicy(QComboBox::InsertAtTop);

  form->addRow("Search text:",
      searchText_ctl);

  connect(searchText_ctl, &QComboBox::currentTextChanged,
      this, &ThisClass::onSearchTextChanged);

  connect(searchText_ctl,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onSearchTextCurrentIndexChanged);


  matchingFlags_ctl =
      add_enum_combobox<Qt::MatchFlag>("Matching type:",
          "",
      [this](Qt::MatchFlags) {
        Q_EMIT parameterChanged();
      });

  caseSensitivity_ctl =
      add_checkbox("Case sensitive",
          "",
          [this] (bool) {
            Q_EMIT parameterChanged();
          });


  invertMatch_ctl =
      add_checkbox("Invert match",
          "",
        [this] (bool) {
          Q_EMIT parameterChanged();
        });

  loadSavedFilters();
}

QThumbnailsQuickFilterOptions::~QThumbnailsQuickFilterOptions()
{
  saveFilters();
}


void QThumbnailsQuickFilterOptions::loadSavedFilters()
{
  QSettings settings;

  QStringList savedFilters =
      settings.value("ThumbnailsQuickFilters",
          QStringList()).toStringList();


  if ( !savedFilters.empty() ) {
    setUpdatingControls(true);

    for( int i = 0, n = std::min(20, (int) savedFilters.size()); i < n; ++i ) {
      searchText_ctl->addItem(savedFilters[i]);
    }

    setUpdatingControls(false);
  }

  matchingFlags_ctl->setCurrentIndex(
      settings.value("ThumbnailsQuickFiltersMatchMethod", 0).
          toInt());

}

void QThumbnailsQuickFilterOptions::saveFilters()
{
  QSettings settings;

  QStringList savedFilters;
  for( int i = 0, n = std::max(32, searchText_ctl->count()); i < n; ++i ) {
    savedFilters.append(searchText_ctl->itemText(i));
  }

  if ( !savedFilters.empty() ) {
    settings.setValue("ThumbnailsQuickFilters",
      savedFilters);
  }

  settings.setValue("ThumbnailsQuickFiltersMatchMethod",
      matchingFlags_ctl->currentIndex());

}

void QThumbnailsQuickFilterOptions::onSearchTextCurrentIndexChanged(int index)
{
  if( index >= 0 && !updatingControls() ) {

    c_update_controls_lock lock(this);

    const QString text =
        searchText_ctl->currentText();

    searchText_ctl->removeItem(index);
    searchText_ctl->insertItem(0, text);
    searchText_ctl->setEditText(text);
  }
}

void QThumbnailsQuickFilterOptions::onSearchTextChanged(const QString & s)
{
  if ( !updatingControls() )  {
    Q_EMIT parameterChanged();
  }

}

void QThumbnailsQuickFilterOptions::setSearchText(const QString & v)
{
  searchText_ctl->setCurrentText(v);
}

QString QThumbnailsQuickFilterOptions::searchText() const
{
  return searchText_ctl->currentText();
}

void QThumbnailsQuickFilterOptions::setMatchingFlags(Qt::MatchFlags v)
{
  const bool inUpdatingControls =
      updatingControls();

  setUpdatingControls(true);

  matchingFlags_ctl->setCurrentItem((Qt::MatchFlag)(int)(v & 0xF));
  caseSensitivity_ctl->setChecked((((uint) v) & Qt::MatchCaseSensitive) != 0);

  setUpdatingControls(inUpdatingControls);
}

Qt::MatchFlags QThumbnailsQuickFilterOptions::matchingFlags() const
{
  Qt::MatchFlags v = matchingFlags_ctl->currentItem();
  if ( caseSensitivity_ctl->isChecked() ) {
    v |= Qt::MatchCaseSensitive;
  }
  return v;
}

void QThumbnailsQuickFilterOptions::setInvertMatch(bool v)
{
  const bool inUpdatingControls =
      updatingControls();

  setUpdatingControls(true);
  invertMatch_ctl->setChecked(v);
  setUpdatingControls(inUpdatingControls);
}

bool QThumbnailsQuickFilterOptions::invertMatch() const
{
  return invertMatch_ctl->isChecked();
}


QThumbnailsQuickFilterDialogBox::QThumbnailsQuickFilterDialogBox(QWidget * parent)
  : Base(parent)
{
  setWindowTitle("Quick filter");

  QVBoxLayout * layout =
      new QVBoxLayout(this);

  layout->addWidget(form_ =
      new QThumbnailsQuickFilterOptions(this));

  connect(form_, &QThumbnailsQuickFilterOptions::parameterChanged,
      this, &ThisClass::parameterChanged);
}

void QThumbnailsQuickFilterDialogBox::setSearchText(const QString & v)
{
  return form_->setSearchText(v);
}

QString QThumbnailsQuickFilterDialogBox::searchText() const
{
  return form_->searchText();
}

void QThumbnailsQuickFilterDialogBox::setMatchingFlags(Qt::MatchFlags v)
{
  return form_->setMatchingFlags(v);
}

Qt::MatchFlags QThumbnailsQuickFilterDialogBox::matchingFlags() const
{
  return form_->matchingFlags();
}

void QThumbnailsQuickFilterDialogBox::setInvertMatch(bool v)
{
  form_->setInvertMatch(v);
}

bool QThumbnailsQuickFilterDialogBox::invertMatch() const
{
  return form_->invertMatch();
}

