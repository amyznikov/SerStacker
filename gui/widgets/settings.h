/*
 * settings.h
 *
 *  Created on: Oct 22, 2018
 *      Author: amyznikov
 */

#ifndef __settings_widgets_h__
#define __settings_widgets_h__

#include <QtWidgets/QtWidgets>
#include <opencv2/opencv.hpp>
#include <core/ssprintf.h>

template<class T>
struct cfmt {
};

template<>
struct cfmt<unsigned char> {
  static constexpr const char * format = "%hhu";
};

template<>
struct cfmt<short> {
  static constexpr const char * format = "%hd";
};

template<>
struct cfmt<unsigned short> {
  static constexpr const char * format = "%hu";
};

template<>
struct cfmt<int> {
  static constexpr const char * format = "%d";
};

template<>
struct cfmt<unsigned int> {
  static constexpr const char * format = "%u";
};

template<>
struct cfmt<long> {
  static constexpr const char * format = "%l";
};

template<>
struct cfmt<unsigned long> {
  static constexpr const char * format = "%lu";
};

template<>
struct cfmt<long long> {
  static constexpr const char * format = "%ll";
};

template<>
struct cfmt<unsigned long long > {
  static constexpr const char * format = "%llu";
};

template<>
struct cfmt<float> {
  static constexpr const char * format = "%f";
};

template<>
struct cfmt<double> {
  static constexpr const char * format = "%lf";
};

#if 0

template<class T>
inline QString toString(const T & v) {
  return QString("%1").arg(v);
}

template<class T>
inline QString toString(const T & x, const T & y) {
  return QString("%1;%2").arg(x).arg(y);
}

template<class T>
inline QString toString(const T & x, const T & y, const T & z) {
  return QString("%1;%2;%3").arg(x).arg(y).arg(z);
}

template<class T>
inline QString toString(const T & x, const T & y, const T & z, const T & w) {
  return QString("%1;%2;%3;%4").arg(x).arg(y).arg(z).arg(w);
}

inline bool fromString(const QString & s, QString * v)
{
  if ( &s != v ) {
    *v = s;
  }
  return true;
}


inline bool fromString(const QString & s, float * v) {
  return sscanf(s.toUtf8().data(), "%f", v) == 1;
}

inline bool fromString(const QString & s, float * x, float * y) {
  return sscanf(s.toUtf8().data(), "%f;%f", x, y) == 2;
}

inline bool fromString(const QString & s, float * x, float * y, float * z) {
  return sscanf(s.toUtf8().data(), "%f;%f;%f", x, y, z) == 3;
}

inline bool fromString(const QString & s, float * x, float * y, float * z, float * w) {
  return sscanf(s.toUtf8().data(), "%f;%f;%f;%f", x, y, z, w) == 4;
}




inline bool fromString(const QString & s, double * v) {
  return sscanf(s.toUtf8().data(), "%lf", v) == 1;
}

inline bool fromString(const QString & s, double * x, double * y) {
  return sscanf(s.toUtf8().data(), "%lf;%lf", x, y) == 2;
}

inline bool fromString(const QString & s, double * x, double * y, double * z) {
  return sscanf(s.toUtf8().data(), "%lf;%lf;%lf", x, y, z) == 3;
}

inline bool fromString(const QString & s, double * x, double * y, double * z, double * w) {
  return sscanf(s.toUtf8().data(), "%lf;%lf;%lf;%lf", x, y, z, w) == 4;
}




inline bool fromString(const QString & s, int * v) {
  return sscanf(s.toUtf8().data(), "%d", v) == 1;
}

inline bool fromString(const QString & s, uint * v) {
  return sscanf(s.toUtf8().data(), "%ud", v) == 1;
}



inline bool fromString(const QString & s, int * x, int * y) {
  return sscanf(s.toUtf8().data(), "%d;%d", x, y) == 2;
}

inline bool fromString(const QString & s, uint * x, uint * y) {
  return sscanf(s.toUtf8().data(), "%u;%u", x, y) == 2;
}



inline bool fromString(const QString & s, int * x, int * y, int * z) {
  return sscanf(s.toUtf8().data(), "%d;%d;%d", x, y, z) == 3;
}

inline bool fromString(const QString & s, uint * x, uint * y, uint * z) {
  return sscanf(s.toUtf8().data(), "%u;%u;%u", x, y, z) == 3;
}



inline bool fromString(const QString & s, int * x, int * y, int * z, int * w) {
  return sscanf(s.toUtf8().data(), "%d;%d;%d;%d", x, y, z, w) == 4;
}

inline bool fromString(const QString & s, uint * x, uint * y, uint * z, uint * w) {
  return sscanf(s.toUtf8().data(), "%u;%u;%u;%u", x, y, z, w) == 4;
}






#ifdef CV_VERSION

template<class T>
inline QString toString(const cv::Point_<T> &  v) {
  return QString("%1;%2").arg(v.x).arg(v.y);
}

inline bool fromString(const QString & s, cv::Point2i * v) {
  cv::Point2i vv;
  return sscanf(s.toUtf8().data(), "%d;%d", &vv.x, &vv.y) == 2 ? *v = vv, true : false;
}

inline bool fromString(const QString & s, cv::Point2f * v) {
  cv::Point2f vv;
  return sscanf(s.toUtf8().data(), "%f;%f", &vv.x, &vv.y) == 2 ? *v = vv, true : false;
}

inline bool fromString(const QString & s, cv::Point2d * v) {
  cv::Point2d vv;
  return sscanf(s.toUtf8().data(), "%lf;%lf", &vv.x, &vv.y) == 2 ? *v = vv, true : false;
}



template<class T>
inline QString toString(const cv::Point3_<T> &  v) {
  return QString("%1;%2;%3").arg(v.x).arg(v.y).arg(v.z);
}
inline bool fromString(const QString & s, cv::Point3i * v) {
  cv::Point3i vv;
  return sscanf(s.toUtf8().data(), "%d;%d;%d", &vv.x, &vv.y, &vv.z) == 3 ? *v = vv, true : false;
}
inline bool fromString(const QString & s, cv::Point3f * v) {
  cv::Point3f vv;
  return sscanf(s.toUtf8().data(), "%f;%f;%f", &vv.x, &vv.y, &vv.z) == 3 ? *v = vv, true : false;
}
inline bool fromString(const QString & s, cv::Point3d * v) {
  cv::Point3d vv;
  return sscanf(s.toUtf8().data(), "%lf;%lf;%lf", &vv.x, &vv.y, &vv.z) == 3 ? *v = vv, true : false;
}


template<class T>
inline QString toString(const cv::Size_<T> &  v) {
  return QString("%1x%2").arg(v.width).arg(v.height);
}

inline bool fromString(const QString & s, cv::Size_<int> * v) {
  cv::Size_<int> vv;
  return sscanf(s.toUtf8().data(), "%d%*[ xX:;,]%d", &vv.width, &vv.height) == 2 ? *v = vv, true: false;
}

inline bool fromString(const QString & s, cv::Size_<int64> * v) {
  cv::Size_<int64> vv;
  return sscanf(s.toUtf8().data(), "%" SCNd64 "%*[ xX:;,]%" SCNd64, &vv.width, &vv.height) == 2 ? *v = vv, true : false;
}

inline bool fromString(const QString & s, cv::Size_<float> * v) {
  cv::Size_<float> vv;
  return sscanf(s.toUtf8().data(), "%f%*[ xX:;,]%f", &vv.width, &vv.height) == 2 ? *v = vv, true : false;
}

inline bool fromString(const QString & s, cv::Size_<double> * v) {
  cv::Size_<double> vv;
  return sscanf(s.toUtf8().data(), "%lf%*[ xX:;,]%lf", &vv.width, &vv.height) == 2 ? *v = vv, true : false;
}


template<class T>
inline QString toString(const cv::Vec<T, 2> &  v) {
  return QString("%1;%2").arg(v[0]).arg(v[1]);
}

template<class T>
inline QString toString(const cv::Vec<T, 3> &  v) {
  return QString("%1;%2;%3").arg(v[0]).arg(v[1]).arg(v[2]);
}

inline bool fromString(const QString & s, cv::Vec<int,2> * v) {
  cv::Vec<int,2> vv;
  return sscanf(s.toUtf8().data(), "%d%*[ :;]%d", &vv[0], &vv[1]) == 2 ? *v = vv, true: false;
}

inline bool fromString(const QString & s, cv::Vec<int,3> * v) {
  cv::Vec<int,3> vv;
  return sscanf(s.toUtf8().data(), "%d%*[ :;]%d%*[ :;]%d", &vv[0], &vv[1], &vv[2]) == 3 ? *v = vv, true: false;
}

inline bool fromString(const QString & s, cv::Vec<float,2> * v) {
  cv::Vec<float,2> vv;
  return sscanf(s.toUtf8().data(), "%f%*[ :;]%f", &vv[0], &vv[1]) == 2 ? *v = vv, true: false;
}

inline bool fromString(const QString & s, cv::Vec<float,3> * v) {
  cv::Vec<float,3> vv;
  return sscanf(s.toUtf8().data(), "%f%*[ :;]%f%*[ :;]%f", &vv[0], &vv[1], &vv[2]) == 3 ? *v = vv, true: false;
}


inline bool fromString(const QString & s, cv::Vec<double,2> * v) {
  cv::Vec<double,2> vv;
  return sscanf(s.toUtf8().data(), "%lf%*[ :;]%lf", &vv[0], &vv[1]) == 2 ? *v = vv, true: false;
}

inline bool fromString(const QString & s, cv::Vec<double,3> * v) {
  cv::Vec<double,3> vv;
  return sscanf(s.toUtf8().data(), "%lf%*[ :;]%lf%*[ :;]%lf", &vv[0], &vv[1], &vv[2]) == 3 ? *v = vv, true: false;
}

inline bool fromString(const QString & text, cv::Scalar * v)
{
  const QByteArray a = text.toUtf8().data();
  const char * s = a.data();

  const int n = sscanf(s, "%lf[ :;]%lf[ :;]%lf[ :;]%lf[ :;]",
      &v->val[0], &v->val[1], &v->val[2], &v->val[3]);

  return n > 0;
}

inline QString toString(const cv::Scalar & v)
{
  return QString("%1;%2;%3;%4").arg(v[0]).arg(v[1]).arg(v[2]).arg(v[3]);
}

#endif


#endif

template<class T>
inline QString toQString(const T & v) {
  return QString(std::string(toString(v)).c_str());
}

template<>
inline QString toQString(const QString & v) {
  return v;
}

template<class T>
inline bool fromString(const QString & text, T * v) {
  return fromString(text.toStdString(), v);
}

template<>
inline bool fromString(const QString & text, QString * v) {
  *v = text;
  return true;
}


template<class T>
inline int fromString(const QString & s, T x[], int nmax)
{
  const char * sp = s.toUtf8().data();
  int n = 0;
  while( n < nmax && sp && *sp && sscanf(sp, cfmt<T>::format, &x[n]) == 1 ) {
    ++n;
    if ( (sp = strpbrk(sp + 1, "; \t")) ) {
      ++sp;
    }
  }

  return n;
}

template<class T>
QString toString(const T x[], int nmax)
{
  QString s;
  for ( int i = 0; i < nmax; ++i ) {
    s.append(toString(x[i]));
    if ( i < nmax - 1 ) {
      s.append(';');
    }
  }
  return s;
}

template<class T>
inline bool fromString(const QString & text, std::vector<T> * v)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
  const QStringList tokens =
      text.split(QRegExp("[ ;:\t\n]"),
          Qt::SkipEmptyParts);
#else
  const QStringList tokens =
      text.split(QRegExp("[ ;:\t\n]"),
          QString::SkipEmptyParts);
#endif

  v->clear();
  v->reserve(tokens.size());

  T value;

  for ( int i = 0, n = tokens.size(); i < n; ++i ) {
    if ( fromString(tokens[i], &value) ) {
      v->emplace_back(value);
    }
    else {
      break;
    }
  }

  return !v->empty();
}

template<class T>
inline QString toQString(const std::vector<T> & v)
{
  QString s;
  for ( uint i = 0, n = v.size(); i < n; ++i ) {
    s.append(QString("%1 ; ").arg(v[i]));
  }
  return s;
}
template<class T>
inline void save_parameter(const QString & prefix, const char * name, const T & value ) {
  QSettings settings;
  settings.setValue(QString("%1/%2").arg(prefix).arg(name), toQString(value));
}

inline void save_parameter(const QString & prefix, const char * name, const QString & value ) {
  QSettings settings;
  settings.setValue(QString("%1/%2").arg(prefix).arg(name), value);
}

inline void save_parameter(const QString & prefix, const char * name, const std::string & value ) {
  QSettings settings;
  settings.setValue(QString("%1/%2").arg(prefix).arg(name), value.c_str());
}

inline void save_parameter(const QString & prefix, const char * name, bool value ) {
  QSettings settings;
  settings.setValue(QString("%1/%2").arg(prefix).arg(name), value);
}

template<class T>
inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, T * value) {
  return fromString(settings.value(QString("%1/%2").arg(prefix).arg(name), "").toString(), value);
}

template<class T> // mainly for custom enums
inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, T * value, const T & defval) {
  * value = fromStdString(settings.value(QString("%1/%2").arg(prefix).arg(name), "").toString().toStdString(), defval);
  return true;
}

inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, bool * value) {
  return *value = settings.value(QString("%1/%2").arg(prefix).arg(name), * value).toBool(), true;
}

inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, int * value) {
  bool ok;
  *value = settings.value(QString("%1/%2").arg(prefix).arg(name), *value).toInt(&ok);
  return ok;
}

inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, float * value) {
  bool ok;
  *value = settings.value(QString("%1/%2").arg(prefix).arg(name), *value).toFloat(&ok);
  return ok;
}

inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, double * value) {
  bool ok;
  *value = settings.value(QString("%1/%2").arg(prefix).arg(name), *value).toDouble(&ok);
  return ok;
}


template<class T>
inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, T * x, T * y) {
  return fromString(settings.value(QString("%1/%2").arg(prefix).arg(name), "").toString(), x, y);
}

template<class T>
inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  T * x, T * y, T * z) {
  return fromString(settings.value(QString("%1/%2").arg(prefix).arg(name), "").toString(), x, y, z);
}

template<class T>
inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  T * x, T * y, T * z, T * w) {
  return fromString(settings.value(QString("%1/%2").arg(prefix).arg(name), "").toString(), x, y, z, w);
}

inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, QString * value) {
  *value = settings.value(QString("%1/%2").arg(prefix).arg(name), *value).toString();
  return true;
}

inline bool load_parameter(const QSettings & settings, const QString & prefix, const char * name, std::string * value) {
  *value = settings.value(QString("%1/%2").arg(prefix).arg(name), value->c_str()).toString().toStdString();
  return true;
}

template<class T>
inline int load_parameter_array(const QSettings & settings, const QString & prefix, const char * name, T * x, int nmax) {
  QString s = settings.value(QString("%1/%2").arg(prefix).arg(name)).toString();
  return s.isEmpty() ? 0 : fromString(s, x, nmax);
}



#endif /* __settings_widgets_h__ */
