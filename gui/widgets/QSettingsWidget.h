/*
 * QSettingsWidget.h
 *
 *  Created on: Jan 11, 2019
 *      Author: amyznikov
 */

#pragma once
#ifndef __QSettingsWidget_h__
#define __QSettingsWidget_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/settings.h>
#include <gui/widgets/QEnumComboBox.h>
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/QExpandableGroupBox.h>
#include <gui/widgets/QSliderSpinBox.h>
#include <type_traits>
#include <functional>
#include <mutex>


class QSettingsWidget
    : public QFrame
{
  Q_OBJECT;

public:
  typedef QSettingsWidget ThisClass;
  typedef QFrame Base;

  struct c_mutex_lock
  {
    QSettingsWidget * this_;

    c_mutex_lock(QSettingsWidget * _this) : this_(_this) {
      this_->lock();
    }

    ~c_mutex_lock() {
      this_->unlock();
    }
  };

  struct c_update_controls_lock {
    ThisClass * _this;
    c_update_controls_lock(ThisClass * obj) : _this(obj) {
      _this->setUpdatingControls(true);
    }
    ~c_update_controls_lock() {
      _this->setUpdatingControls(false);
    }
  };


  QSettingsWidget(const QString & prefix, QWidget * parent = Q_NULLPTR);

  void setSettingsPrefix(const QString & v);
  const QString& settingsPrefix() const;

  void set_mutex(std::mutex * mtx);
  std::mutex* mutex();

  void loadSettings(QSettings & settings);

  void updateControls();
  virtual void setUpdatingControls(bool v);
  virtual bool updatingControls();

public Q_SLOTS:
  void loadParameters();

Q_SIGNALS:
  void parameterChanged();
  void populatecontrols();

protected:
  virtual void onload(QSettings & settings);
  virtual void onupdatecontrols();
  virtual void lock();
  virtual void unlock();

public:

  /////////////////////////////////////////////////////////////////////
  void removeWidget( QWidget * w)
  {
    if ( form ) {
      form->removeWidget(w);
    }
  }

  void insertWidget(int row, QWidget * w)
  {
    if ( form ) {
      form->insertRow(row, w);
    }
  }

  /////////////////////////////////////////////////////////////////////

  template<class T>
  QNumberEditBox * add_numeric_box(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn = std::function<void(T)>())
  {
    QNumberEditBox * ctl = new QNumberEditBox(this);
    if ( !tooltip.isEmpty() ) {
      ctl->setToolTip(tooltip);
    }
    form->addRow(name, ctl);


    if ( setfn ) {
      QObject::connect(ctl, &QNumberEditBox::textChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() && setfn ) {
              T v;
              if ( fromString(ctl->text(), &v) ) {
                c_mutex_lock lock(this);
                setfn(v);
              }
            }
          });
    }
    return ctl;
  }

  template<class T>
  QNumberEditBox * add_numeric_box(QFormLayout * form, const QString & name,
      const std::function<void(T)> & setfn = std::function<void(T)>())
  {
    return add_numeric_box<T>(form, name, QString(), setfn);
  }


  template<class T>
  QNumberEditBox * add_numeric_box(QFormLayout * form, const QString & name,
      const std::function<void(T)> & setfn, const std::function<bool(T*)> & getfn)
  {
    QNumberEditBox * ctl =
        add_numeric_box(form, name, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                T v;
                if ( getfn(&v) ) {
                  ctl->setValue(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  template<class T>
  QNumberEditBox * add_numeric_box(const QString & name, const std::function<void(T)> & setfn = std::function<void(T)>() )
  {
    return add_numeric_box<T>(this->form, name, setfn);
  }

  template<class T>
  QNumberEditBox * add_numeric_box(const QString & name, const QString & tooltip, const std::function<void(T)> & setfn = std::function<void(T)>() )
  {
    return add_numeric_box<T>(this->form, name, tooltip, setfn);
  }

  template<class T>
  QNumberEditBox * add_numeric_box(const QString & name,const std::function<void(T)> & setfn , const std::function<bool(T*)> & getfn)
  {
    return add_numeric_box<T>(this->form, name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  QLineEditBox* add_textbox(QFormLayout * form, const QString & name,
      const std::function<void(const QString&)> & setfn = std::function<void(const QString&)>(),
      const std::function<bool(QString*)> & getfn = std::function<bool(QString*)>())
  {
    QLineEditBox *ctl =
        new QLineEditBox();

    form->addRow(name, ctl);

    if( setfn ) {
      QObject::connect(ctl, &QLineEditBox::textChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->text());
            }
          });
    }

    if( getfn ) {
      QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            QString v;
            if ( getfn(&v) ) {
              ctl->setText(v);
            }
          });
    }

//    QObject::connect(ctl, &QObject::destroyed,
//        [this, ctl]() {
//          ctl->disconnect(this);
//          // QObject::disconnect(ctl, nullptr, this, nullptr);
//        });

    return ctl;
  }

  QLineEditBox* add_textbox(const QString & name,
      const std::function<void(const QString&)> & setfn = std::function<void(const QString&)>(),
      const std::function<bool(QString*)> & getfn = std::function<bool(QString*)>())
  {
    return add_textbox(form, name, setfn, getfn);
  }

//  QLineEditBox* add_textbox(const QString & name,
//      const std::function<void(const QString&)> & setfn = std::function<void(const QString&)>())
//  {
//    return add_textbox(this->form, name, setfn);
//  }
//
//  QLineEditBox* add_textbox(const QString & name, const std::function<void(const QString&)> & setfn,
//      const std::function<bool(QString *)> & getfn = std::function<bool( QString*)>())
//  {
//    return add_textbox(this->form, name, setfn);
//  }

  /////////////////////////////////////////////////////////////////////

  QCheckBox* add_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(bool)> & setfn = std::function<void(bool)>())
  {
    QCheckBox *ctl = new QCheckBox(this);
    form->addRow(name, ctl);

    if( setfn ) {
      QObject::connect(ctl, &QCheckBox::stateChanged,
          [this, setfn](int state) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(state==Qt::Checked);
            }
          });
    }

    return ctl;
  }

  QCheckBox* add_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(bool)> & setfn , const std::function<bool(bool*)> & getfn)
  {
    QCheckBox *ctl = add_checkbox(form, name, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                bool v;
                if ( getfn(&v) ) {
                  ctl->setChecked(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  QCheckBox * add_checkbox(const QString & name, const std::function<void(bool)> & setfn = std::function<void(bool)>() )
  {
    return add_checkbox(this->form, name, setfn);
  }

  QCheckBox * add_checkbox(const QString & name, const std::function<void(bool)> & setfn, const std::function<bool(bool*)> & getfn)
  {
    return add_checkbox(this->form, name, setfn, getfn);
  }


  /////////////////////////////////////////////////////////////////////


  QCheckBox* add_named_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(bool)> & setfn = std::function<void(bool)>())
  {
    QCheckBox *ctl = new QCheckBox(name, this);
    form->addRow(ctl);

    if( setfn ) {
      QObject::connect(ctl, &QCheckBox::stateChanged,
          [this, setfn](int state) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(state==Qt::Checked);
            }
          });
    }

    return ctl;
  }

  QCheckBox* add_named_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(bool)> & setfn , const std::function<bool(bool*)> & getfn)
  {
    QCheckBox *ctl = add_checkbox(form, name, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                bool v;
                if ( getfn(&v) ) {
                  ctl->setChecked(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  QCheckBox * add_named_checkbox(const QString & name, const std::function<void(bool)> & setfn = std::function<void(bool)>() )
  {
    return add_checkbox(this->form, name, setfn);
  }

  QCheckBox * add_named_checkbox(const QString & name, const std::function<void(bool)> & setfn, const std::function<bool(bool*)> & getfn)
  {
    return add_checkbox(this->form, name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(QFormLayout * form, const QString & name,
      const std::function<void(EnumType)> & setfn = std::function<void(EnumType)>())
  {
    typedef QEnumComboBox<EnumType> CombomoxType;

    CombomoxType * ctl = new CombomoxType(this);
    form->addRow(name, ctl);

    if( setfn ) {
      QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->currentItem());
            }
          });
    }

    return ctl;
  }

  template<class EnumType>
  QEnumComboBox<EnumType>* add_enum_combobox(QFormLayout * form, const QString & name,
      const std::function<void(EnumType)> & setfn, const std::function<bool(EnumType*)> & getfn)
  {
    QEnumComboBox<EnumType> *ctl =
        add_enum_combobox(form, name, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                EnumType v;
                if ( getfn(&v) ) {
                  ctl->setCurrentItem(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(const QString & name,
      const std::function<void(EnumType)> & setfn = std::function<void(EnumType)>())
  {
    return add_enum_combobox<EnumType>(this->form, name, setfn);
  }

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(const QString & name,
      const std::function<void(EnumType)> & setfn, const std::function<bool(EnumType*)> & getfn)
  {
    return add_enum_combobox<EnumType>(this->form, name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class ComboBoxType = QComboBox>
  ComboBoxType* add_combobox(QFormLayout * form, const QString & name,
      const std::function<void(int, ComboBoxType*)> & setfn = std::function<void(int, ComboBoxType*)>())
  {
    ComboBoxType *ctl = new ComboBoxType();
    ctl->setFocusPolicy(Qt::StrongFocus);
    form->addRow(name, ctl);

    if( setfn ) {
      QObject::connect(ctl,
          static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          [this, ctl, setfn](int v) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(v, ctl);
            }
          });
    }

    return ctl;
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType* add_combobox(QFormLayout * form, const QString & name,
      const std::function<void(int, ComboBoxType*)> & setfn,
      const std::function<bool(int*, ComboBoxType*)> & getfn)
  {
    ComboBoxType *ctl =
        add_combobox(form, name, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                int v;
                if ( getfn(&v, ctl) ) {
                  ctl->setCurrentIndex(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  template<typename ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(const QString & name, const std::function<void(int, ComboBoxType*)> & setfn =
      std::function<void(int, ComboBoxType*)>())
  {
    return add_combobox<ComboBoxType>(this->form, name, setfn);
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(const QString & name, const std::function<void(int, ComboBoxType*)> & setfn,
      const std::function<bool(int*, ComboBoxType*)> & getfn)
  {
    return add_combobox<ComboBoxType>(this->form, name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////
  QSpinBox* add_spinbox(QFormLayout * form, const QString & name, const std::function<void(int)> & setfn =
      std::function<void(int)>())
  {
    QSpinBox *ctl = new QSpinBox();
    ctl->setKeyboardTracking(false);
    ctl->setFocusPolicy(Qt::StrongFocus);

    form->addRow(name, ctl);

    if( setfn ) {
      QObject::connect(ctl,
          static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          [this, ctl, setfn](int v) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(v);
            }
          });
    }

    return ctl;
  }

  QSpinBox* add_spinbox(QFormLayout * form, const QString & name, const std::function<void(int)> & setfn,
      const std::function<bool(int*)> & getfn)
  {
    QSpinBox *ctl =
        add_spinbox(form, name, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                int v;
                if ( getfn(&v) ) {
                  ctl->setValue(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  QSpinBox* add_spinbox(const QString & name, const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    return add_spinbox(this->form, name, setfn);
  }

  QSpinBox* add_spinbox(const QString & name, const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    return add_spinbox(this->form, name, setfn, getfn);
  }


  /////////////////////////////////////////////////////////////////////
  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(QFormLayout * form, const QString & name,
      const std::function<void(T)> & setfn = std::function<void(T)>())
  {
    typename QSliderSpinBox<T>::Type *ctl = new typename QSliderSpinBox<T>::Type();

    form->addRow(name, ctl);

    if( setfn ) {

      QObject::connect(ctl,
          &QSliderSpinBox<T>::Type::valueChanged,
          [this, ctl, setfn](T v) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(v);
            }
          });

    }

    return ctl;
  }

  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(QFormLayout * form, const QString & name,
      const std::function<void(T)> & setfn, const std::function<bool(T*)> & getfn)
  {
    typename QSliderSpinBox<T>::Type *ctl =
        add_sliderspinbox<T>(form, name, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                T v;
                if ( getfn(&v) ) {
                  ctl->setValue(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }


  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(const QString & name,
      const std::function<void(T)> & setfn = std::function<void(T)>())
  {
    return add_sliderspinbox<T>(this->form, name, setfn);
  }

  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(const QString & name,
      const std::function<void(T)> & setfn, const std::function<bool(T*)> & getfn)
  {
    return add_sliderspinbox<T>(this->form, name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class T, class S, class G>
  typename std::enable_if<std::is_arithmetic<T>::value && !std::is_same<T, bool>::value,
  QNumberEditBox*>::type add_ctl(const QString & name, const S & setfn, const G & getfn)
  {
    return add_numeric_box<T>(name, setfn, getfn);
  }

  template<class T, class S, class G>
  typename std::enable_if<std::is_same<T, bool>::value,
  QCheckBox*>::type add_ctl(const QString & name, const S & setfn, const G & getfn)
  {
    return add_checkbox(name, setfn, getfn);
  }

  template<class T, class S, class G>
  typename std::enable_if<std::is_enum<T>::value,
  QEnumComboBox<T>*>::type add_ctl(const QString & name, const S & setfn, const G & getfn)
  {
    return add_enum_combobox<T>(name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  QExpandableGroupBox* add_expandable_groupbox(QFormLayout * form, const QString & title, QWidget * ctl = nullptr,
      int stretch = 0, Qt::Alignment alignment = Qt::Alignment())
  {
    QExpandableGroupBox *gbox = new QExpandableGroupBox(title, ctl, stretch, alignment);
    form->addRow(gbox);
    return gbox;
  }

  QExpandableGroupBox * add_expandable_groupbox(const QString & title, QWidget * ctl = nullptr,
      int stretch = 0, Qt::Alignment alignment = Qt::Alignment())
  {
    return add_expandable_groupbox(this->form, title, ctl, stretch, alignment);
  }

  /////////////////////////////////////////////////////////////////////
  QToolButton * add_tool_button(QFormLayout * form, const QString & name, const QIcon & icon,
      const std::function<void(bool)> & onclick = std::function<void(bool)>())
  {
    QToolButton * ctl = new QToolButton(this);
    ctl->setToolButtonStyle(name.isEmpty() ? Qt::ToolButtonStyle::ToolButtonIconOnly : Qt::ToolButtonStyle::ToolButtonTextBesideIcon);
    ctl->setIconSize(QSize(16,16));
    ctl->setText(name);
    ctl->setIcon(icon);

    form->addRow(ctl);

    if( onclick ) {

      QMetaObject::Connection conn =
          QObject::connect(ctl,
              &QToolButton::clicked,
              onclick);

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });

    }

    return ctl;
  }

  QToolButton * add_tool_button(const QString & name, const QIcon & icon,
      const std::function<void(bool)> & onclick = std::function<void(bool)>())
  {
    return add_tool_button(form, name, icon, onclick);
  }

  QToolButton * add_tool_button(const QString & name,
      const std::function<void(bool)> & onclick = std::function<void(bool)>())
  {
    return add_tool_button(form, name, QIcon(), onclick);
  }

  /////////////////////////////////////////////////////////////////////

  template<class WidgetType>
  WidgetType * add_widget(QFormLayout * form, const QString & name = QString())
  {
    WidgetType * ctl = new WidgetType(this);
    if ( !name.isEmpty() ) {
      form->addRow(name, ctl);
    }
    else {
      form->addRow(ctl);
    }
    return ctl;
  }

  template<class WidgetType>
  WidgetType * add_widget(const QString & name = QString())
  {
    return add_widget<WidgetType>(this->form, name);
  }

  /////////////////////////////////////////////////////////////////////


protected:
  QString PREFIX;
  QFormLayout *form = Q_NULLPTR;

private:
  std::mutex *mtx_ = Q_NULLPTR;
  int updatingControls_ = 0;
};

#endif /* __QSettingsWidget_h__ */
