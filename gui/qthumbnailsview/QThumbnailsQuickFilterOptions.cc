/*
 * QThumbnailsQuickFilterOptions.cc
 *
 *  Created on: Jul 6, 2021
 *      Author: amyznikov
 */

#include "QThumbnailsQuickFilterOptions.h"
#include <core/debug.h>


const struct QtMatchingFlags_desc QtMatchingFlags[] = {
    {"Contains", Qt::MatchContains },
    {"Wildcard", Qt::MatchWildcard},
    {"StartsWith", Qt::MatchStartsWith},
    {"EndsWith", Qt::MatchEndsWith},
    {"Exact", Qt::MatchExactly},
    {"RegExp", Qt::MatchRegularExpression},
    {nullptr, (Qt::MatchFlag)(0)}
};


QString toString(Qt::MatchFlags v)
{
  v = v & 0xF;
  if ( v == Qt::MatchRegExp ) {
    v = Qt::MatchRegularExpression;
  }
  else if ( v == Qt::MatchFixedString ) {
    v = Qt::MatchExactly;
  }

  for ( uint i = 0; QtMatchingFlags[i].name; ++i ) {
    if ( QtMatchingFlags[i].value == v ) {
      return QtMatchingFlags[i].name;
    }
  }
  return "";
}

Qt::MatchFlags fromString(const QString & s, Qt::MatchFlags defval)
{
  const QByteArray a = s.toUtf8();
  const char * cstr = a.data();

  for ( uint i = 0; QtMatchingFlags[i].name; ++i ) {
    if ( strcasecmp(QtMatchingFlags[i].name, cstr) == 0 ) {
      return QtMatchingFlags[i].value;
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

     const uint matchType = flags & 0x0F;
     const Qt::CaseSensitivity cs = (flags & Qt::MatchCaseSensitive) ? Qt::CaseSensitive : Qt::CaseInsensitive;

     switch ( matchType ) {

     case Qt::MatchFixedString :
       case Qt::MatchExactly : {
       matchStatus = pattern.compare(text, cs) == 0;
       break;
     }

     case Qt::MatchRegExp : {
       matchStatus = QRegExp(pattern, cs, QRegExp::RegExp).exactMatch(text);
       break;
     }

     case Qt::MatchWildcard : {
       matchStatus = QRegExp(pattern, cs, QRegExp::Wildcard).exactMatch(text);
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


QThumbnailsQuickFilterOptions::QThumbnailsQuickFilterOptions(QWidget * parent)
  : Base("QThumbnailsQuickFilterOptions", parent)
{

  const auto emit_parameterChanged =
      [this]() {
        emit parameterChanged();
      };

  searchText_ctl = new QComboBox(this);
  searchText_ctl->setEditable(true);
  searchText_ctl->setAutoCompletion(true);
  searchText_ctl->setMinimumContentsLength(32);
  searchText_ctl->setInsertPolicy(QComboBox::InsertAtTop);

  form->addRow("Search text:",
      searchText_ctl);

  connect(searchText_ctl, &QComboBox::currentTextChanged,
      this, &ThisClass::onSearchTextChanged);

  add_combobox(form, "Matching type:",
      matchingFlags_ctl = new QThumbnailsQuickFilterMatchingFlagsCombo(this),
      emit_parameterChanged);

  caseSensitivity_ctl =
      add_checkbox("Case sensitive",
          [this] (int) {
            emit parameterChanged();
          });

}


void QThumbnailsQuickFilterOptions::onSearchTextChanged(const QString & s)
{
  if ( !updatingControls() )  {
    emit parameterChanged();
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
  matchingFlags_ctl->setCurrentItem((Qt::MatchFlags)(v & 0xF));
  caseSensitivity_ctl->setChecked((((uint) v) & Qt::MatchCaseSensitive) != 0);
}

Qt::MatchFlags QThumbnailsQuickFilterOptions::matchingFlags() const
{
  Qt::MatchFlags v = matchingFlags_ctl->currentItem();
  if ( caseSensitivity_ctl->isChecked() ) {
    v |= Qt::MatchCaseSensitive;
  }
  return v;
}


QThumbnailsQuickFilterDialogBox::QThumbnailsQuickFilterDialogBox(QWidget * parent)
  : Base(parent)
{
  QVBoxLayout * layout =
      new QVBoxLayout(this);

  setWindowTitle("Quick filter");
  layout->addWidget(form_ = new QThumbnailsQuickFilterOptions(this));

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


