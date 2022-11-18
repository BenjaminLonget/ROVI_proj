/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/SamplePlugin.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SamplePlugin_t {
    const uint offsetsAndSize[50];
    char stringdata0[261];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_SamplePlugin_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_SamplePlugin_t qt_meta_stringdata_SamplePlugin = {
    {
QT_MOC_LITERAL(0, 12), // "SamplePlugin"
QT_MOC_LITERAL(13, 10), // "btnPressed"
QT_MOC_LITERAL(24, 0), // ""
QT_MOC_LITERAL(25, 5), // "timer"
QT_MOC_LITERAL(31, 8), // "getImage"
QT_MOC_LITERAL(40, 11), // "get25DImage"
QT_MOC_LITERAL(52, 20), // "stateChangedListener"
QT_MOC_LITERAL(73, 21), // "rw::kinematics::State"
QT_MOC_LITERAL(95, 5), // "state"
QT_MOC_LITERAL(101, 15), // "checkCollisions"
QT_MOC_LITERAL(117, 11), // "Device::Ptr"
QT_MOC_LITERAL(129, 6), // "device"
QT_MOC_LITERAL(136, 5), // "State"
QT_MOC_LITERAL(142, 17), // "CollisionDetector"
QT_MOC_LITERAL(160, 8), // "detector"
QT_MOC_LITERAL(169, 1), // "Q"
QT_MOC_LITERAL(171, 1), // "q"
QT_MOC_LITERAL(173, 20), // "createPathRRTConnect"
QT_MOC_LITERAL(194, 4), // "from"
QT_MOC_LITERAL(199, 2), // "to"
QT_MOC_LITERAL(202, 6), // "extend"
QT_MOC_LITERAL(209, 7), // "maxTime"
QT_MOC_LITERAL(217, 21), // "printProjectionMatrix"
QT_MOC_LITERAL(239, 11), // "std::string"
QT_MOC_LITERAL(251, 9) // "frameName"

    },
    "SamplePlugin\0btnPressed\0\0timer\0getImage\0"
    "get25DImage\0stateChangedListener\0"
    "rw::kinematics::State\0state\0checkCollisions\0"
    "Device::Ptr\0device\0State\0CollisionDetector\0"
    "detector\0Q\0q\0createPathRRTConnect\0"
    "from\0to\0extend\0maxTime\0printProjectionMatrix\0"
    "std::string\0frameName"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SamplePlugin[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   62,    2, 0x08,    1 /* Private */,
       3,    0,   63,    2, 0x08,    2 /* Private */,
       4,    0,   64,    2, 0x08,    3 /* Private */,
       5,    0,   65,    2, 0x08,    4 /* Private */,
       6,    1,   66,    2, 0x08,    5 /* Private */,
       9,    4,   69,    2, 0x08,    7 /* Private */,
      17,    4,   78,    2, 0x08,   12 /* Private */,
      22,    1,   87,    2, 0x08,   17 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Bool, 0x80000000 | 10, 0x80000000 | 12, 0x80000000 | 13, 0x80000000 | 15,   11,    8,   14,   16,
    QMetaType::Void, 0x80000000 | 15, 0x80000000 | 15, QMetaType::Double, QMetaType::Double,   18,   19,   20,   21,
    QMetaType::Void, 0x80000000 | 23,   24,

       0        // eod
};

void SamplePlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SamplePlugin *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->btnPressed(); break;
        case 1: _t->timer(); break;
        case 2: _t->getImage(); break;
        case 3: _t->get25DImage(); break;
        case 4: _t->stateChangedListener((*reinterpret_cast< std::add_pointer_t<rw::kinematics::State>>(_a[1]))); break;
        case 5: { bool _r = _t->checkCollisions((*reinterpret_cast< std::add_pointer_t<Device::Ptr>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<State>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<CollisionDetector>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 6: _t->createPathRRTConnect((*reinterpret_cast< std::add_pointer_t<Q>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[4]))); break;
        case 7: _t->printProjectionMatrix((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject SamplePlugin::staticMetaObject = { {
    QMetaObject::SuperData::link<rws::RobWorkStudioPlugin::staticMetaObject>(),
    qt_meta_stringdata_SamplePlugin.offsetsAndSize,
    qt_meta_data_SamplePlugin,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_SamplePlugin_t
, QtPrivate::TypeAndForceComplete<SamplePlugin, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const rw::kinematics::State &, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<Device::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<const State &, std::false_type>, QtPrivate::TypeAndForceComplete<const CollisionDetector &, std::false_type>, QtPrivate::TypeAndForceComplete<const Q &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>


>,
    nullptr
} };


const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(this);
    return rws::RobWorkStudioPlugin::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rws::RobWorkStudioPlugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 8;
    }
    return _id;
}

QT_PLUGIN_METADATA_SECTION
static constexpr unsigned char qt_pluginMetaData_SamplePlugin[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', '!',
    // metadata version, Qt version, architectural requirements
    0, QT_VERSION_MAJOR, QT_VERSION_MINOR, qPluginArchRequirements(),
    0xbf, 
    // "IID"
    0x02,  0x78,  0x2a,  'd',  'k',  '.',  's',  'd', 
    'u',  '.',  'm',  'i',  'p',  '.',  'R',  'o', 
    'b',  'w',  'o',  'r',  'k',  '.',  'R',  'o', 
    'b',  'W',  'o',  'r',  'k',  'S',  't',  'u', 
    'd',  'i',  'o',  'P',  'l',  'u',  'g',  'i', 
    'n',  '/',  '0',  '.',  '1', 
    // "className"
    0x03,  0x6c,  'S',  'a',  'm',  'p',  'l',  'e', 
    'P',  'l',  'u',  'g',  'i',  'n', 
    // "MetaData"
    0x04,  0xa3,  0x6c,  'd',  'e',  'p',  'e',  'n', 
    'd',  'e',  'n',  'c',  'i',  'e',  's',  0x80, 
    0x64,  'n',  'a',  'm',  'e',  0x6b,  'p',  'l', 
    'u',  'g',  'i',  'n',  'U',  'I',  'a',  'p', 
    'p',  0x67,  'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x65,  '1',  '.',  '0',  '.',  '0', 
    0xff, 
};
QT_MOC_EXPORT_PLUGIN(SamplePlugin, SamplePlugin)

QT_WARNING_POP
QT_END_MOC_NAMESPACE
