# 2д библиотека физики
special thanks to:
https://github.com/RandyGaul/ImpulseEngine
https://habr.com/ru/post/336908/
https://github.com/c-krit/ferox/tree/main/ferox
https://github.com/MarginallyClever/processingPhysics


why single header:
https://stackoverflow.com/questions/12671383/benefits-of-header-only-libraries

идея: многие движки используют сложную систему коллбэков или хранения данных в void* userdata(как BOX2d) для связи игровых объектов и физ. тел. 
у меня другой подход: после каждой итерации пользователю доступен список манифолдов с информацией о столкновениях, которые он сам может проверить в цикле и обработать.

пример из демо: 
```
PhysicsStep(state);
for (int i = 0; i < state->manifoldsCount; i++)
{
    PhysicsManifoldData m = state->manifolds[i];
    // printf("%d",m->contactsCount);
    if (m.bodyA == circle && m.contactsCount 0 && m.bodyB != floor)
    {
        m.bodyB->velocity = m.normal;
    }
    else if (m.bodyB == circle && m.contactsCoun> 0 && m.bodyA != floor)
    {
        m.bodyA->velocity = m.normal;
    }
}
```


планы:
сделать биндинг для жавы(думаю как)