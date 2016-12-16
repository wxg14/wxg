/****************************************************************************
** Meta object code from reading C++ file 'MPU6050Calculator.h'
**
** Created: Tue Dec 6 16:21:48 2016
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../MPU6050Calculator.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MPU6050Calculator.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MPU6050Calculator[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      31,   19,   18,   18, 0x05,
      87,   68,   18,   18, 0x05,
     132,  119,   18,   18, 0x05,
     162,  119,   18,   18, 0x05,
     229,  192,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
     332,  280,   18,   18, 0x0a,
     454,  420,   18,   18, 0x0a,
     570,  524,   18,   18, 0x0a,
     654,  623,   18,   18, 0x2a,
     741,  702,   18,   18, 0x0a,
     811,  789,   18,   18, 0x0a,
     861,  849,   18,   18, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MPU6050Calculator[] = {
    "MPU6050Calculator\0\0aid,x,y,z,w\0"
    "outQuat(int,float,float,float,float)\0"
    "aid,yaw,pitch,roll\0outAngle(int,float,float,float)\0"
    "aid,cx,cy,cz\0outCom(int,float,float,float)\0"
    "outPos(int,float,float,float)\0"
    "chipID,xmax,ymax,zmax,xmin,ymin,zmin\0"
    "updatecom(int,float,float,float,float,float,float)\0"
    "chipID,ax,ay,az,gx,gy,gz,cx,cy,cz,gravx,gravy,gravz\0"
    "newAgcGrav(int,double,double,double,double,double,double,int,int,int,f"
    "loat,float,float)\0"
    "chipID,ax,ay,az,gx,gy,gz,cx,cy,cz\0"
    "newAgcData(int,double,double,double,double,double,double,int,int,int)\0"
    "chipID,yaw,pitch,roll,moveflag,weightnessless\0"
    "newYawPitchRoll(int,double,double,double,short,bool)\0"
    "chipID,yaw,pitch,roll,moveflag\0"
    "newYawPitchRoll(int,double,double,double,short)\0"
    "chipID,x,y,z,w,moveflag,weightnessless\0"
    "newQuat(int,float,float,float,float,short,bool)\0"
    "chipID,posx,posy,posz\0"
    "newPosition(int,double,double,double)\0"
    "fir,sec,thd\0bendData(float,float,float)\0"
};

void MPU6050Calculator::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MPU6050Calculator *_t = static_cast<MPU6050Calculator *>(_o);
        switch (_id) {
        case 0: _t->outQuat((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])),(*reinterpret_cast< float(*)>(_a[5]))); break;
        case 1: _t->outAngle((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4]))); break;
        case 2: _t->outCom((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4]))); break;
        case 3: _t->outPos((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4]))); break;
        case 4: _t->updatecom((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])),(*reinterpret_cast< float(*)>(_a[5])),(*reinterpret_cast< float(*)>(_a[6])),(*reinterpret_cast< float(*)>(_a[7]))); break;
        case 5: _t->newAgcGrav((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6])),(*reinterpret_cast< double(*)>(_a[7])),(*reinterpret_cast< int(*)>(_a[8])),(*reinterpret_cast< int(*)>(_a[9])),(*reinterpret_cast< int(*)>(_a[10])),(*reinterpret_cast< float(*)>(_a[11])),(*reinterpret_cast< float(*)>(_a[12])),(*reinterpret_cast< float(*)>(_a[13]))); break;
        case 6: _t->newAgcData((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6])),(*reinterpret_cast< double(*)>(_a[7])),(*reinterpret_cast< int(*)>(_a[8])),(*reinterpret_cast< int(*)>(_a[9])),(*reinterpret_cast< int(*)>(_a[10]))); break;
        case 7: _t->newYawPitchRoll((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< short(*)>(_a[5])),(*reinterpret_cast< bool(*)>(_a[6]))); break;
        case 8: _t->newYawPitchRoll((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< short(*)>(_a[5]))); break;
        case 9: _t->newQuat((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])),(*reinterpret_cast< float(*)>(_a[5])),(*reinterpret_cast< short(*)>(_a[6])),(*reinterpret_cast< bool(*)>(_a[7]))); break;
        case 10: _t->newPosition((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        case 11: _t->bendData((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MPU6050Calculator::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MPU6050Calculator::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_MPU6050Calculator,
      qt_meta_data_MPU6050Calculator, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MPU6050Calculator::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MPU6050Calculator::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MPU6050Calculator::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MPU6050Calculator))
        return static_cast<void*>(const_cast< MPU6050Calculator*>(this));
    return QObject::qt_metacast(_clname);
}

int MPU6050Calculator::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void MPU6050Calculator::outQuat(int _t1, float _t2, float _t3, float _t4, float _t5)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MPU6050Calculator::outAngle(int _t1, float _t2, float _t3, float _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MPU6050Calculator::outCom(int _t1, float _t2, float _t3, float _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MPU6050Calculator::outPos(int _t1, float _t2, float _t3, float _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void MPU6050Calculator::updatecom(int _t1, float _t2, float _t3, float _t4, float _t5, float _t6, float _t7)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)), const_cast<void*>(reinterpret_cast<const void*>(&_t7)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
