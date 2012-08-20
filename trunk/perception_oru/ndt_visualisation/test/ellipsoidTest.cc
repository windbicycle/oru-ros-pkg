


glTranslate(position.x, position.y, position.z);
glRotate(angle, axis.x, axis.y, axis.z);
glScale3f(radius.x, radius.y, radius.z);
DrawUnitSphere();

GLUquadricObj* obj = gluNewQuadric();
glScale3f(radiusX, radiusY, radiusZ);
gluSphere(obj, 1.0, 10, 10);
gluDeleteQuadric(obj);