/*
 * QDataAnnotationSelectorCtrl.h
 *
 *  Created on: Jul 3, 2025
 *      Author: gandriim
 */

#ifndef __QDataAnnotationSelectorCtrl_h__
#define __QDataAnnotationSelectorCtrl_h__

#include <QtWidgets/QtWidgets>
#include <core/data_annotation/c_data_annotation_labels.h>

class QDataAnnotationSelectorCtrl :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QDataAnnotationSelectorCtrl ThisClass;
  typedef QWidget Base;

  QDataAnnotationSelectorCtrl(QWidget * parent = nullptr);

  int num_colormaps() const
  {
    return _selectors.size();
  }

  void setAnnotationLabel(int cmap, int label);
  bool getAnnotationLabel(int cmap, int *label) const;

Q_SIGNALS:
  void annotationLabelChanged(int cmap, int label);

protected:
  void updateColormaps();

protected:
  QFormLayout * _form = nullptr;
  QList<QComboBox *> _selectors;

};

#endif /* __QDataAnnotationSelectorCtrl_h__ */
