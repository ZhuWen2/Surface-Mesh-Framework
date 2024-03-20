/****************************************************************************
** Meta object code from reading C++ file 'QGLViewerWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.13.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/MeshViewer/QGLViewerWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QGLViewerWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.13.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QGLViewerWidget_t {
    QByteArrayData data[11];
    char stringdata0[139];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QGLViewerWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QGLViewerWidget_t qt_meta_stringdata_QGLViewerWidget = {
    {
QT_MOC_LITERAL(0, 0, 15), // "QGLViewerWidget"
QT_MOC_LITERAL(1, 16, 18), // "setDrawMode_signal"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 13), // "UpdateGL_Slot"
QT_MOC_LITERAL(4, 50, 25), // "rotate_mesh_by_angle_slot"
QT_MOC_LITERAL(5, 76, 5), // "angle"
QT_MOC_LITERAL(6, 82, 22), // "set_ORTHOTROPIC2D_slot"
QT_MOC_LITERAL(7, 105, 7), // "w_left_"
QT_MOC_LITERAL(8, 113, 8), // "w_right_"
QT_MOC_LITERAL(9, 122, 9), // "w_bottom_"
QT_MOC_LITERAL(10, 132, 6) // "w_top_"

    },
    "QGLViewerWidget\0setDrawMode_signal\0\0"
    "UpdateGL_Slot\0rotate_mesh_by_angle_slot\0"
    "angle\0set_ORTHOTROPIC2D_slot\0w_left_\0"
    "w_right_\0w_bottom_\0w_top_"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QGLViewerWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   37,    2, 0x0a /* Public */,
       4,    1,   38,    2, 0x0a /* Public */,
       6,    4,   41,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,    5,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    7,    8,    9,   10,

       0        // eod
};

void QGLViewerWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QGLViewerWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setDrawMode_signal((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->UpdateGL_Slot(); break;
        case 2: _t->rotate_mesh_by_angle_slot((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->set_ORTHOTROPIC2D_slot((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QGLViewerWidget::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewerWidget::setDrawMode_signal)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QGLViewerWidget::staticMetaObject = { {
    &QGLWidget::staticMetaObject,
    qt_meta_stringdata_QGLViewerWidget.data,
    qt_meta_data_QGLViewerWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QGLViewerWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QGLViewerWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QGLViewerWidget.stringdata0))
        return static_cast<void*>(this);
    return QGLWidget::qt_metacast(_clname);
}

int QGLViewerWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void QGLViewerWidget::setDrawMode_signal(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
