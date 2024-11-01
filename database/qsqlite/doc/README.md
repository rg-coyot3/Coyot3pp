# QSqlite


QSqlite connector


## INTRODUCTION

Sqlite is a good solution for minimal systems with complex enough datasets, when external robust database systems (such PostgreSQL) are not available. Sqlite is a minimal file-system-based database that permit to the developper to see its project grow, and imagine a future migration to a more complex and robust system and conserve all the data generated along the lifecycle of the project.

My main objective with this connector was to be able to work directly with the data-models as if they were DAO. While I was making projects, I realized that the *core* of any project would not have any interest in the database itself, BUT to *just simply* know if the access to the database is *up* or *down*... and nothing else. And I also wanted to use the adventages of using the `model-class` preprocessor macros that I defined at the `Cor3`.

 For the projects I made I identified a stage of maturity where I really had nothing but some little real needs:

* read the current stored data at the database. Mainly to read configuration data.
* insert new data at the tables. Mainly logs and traces generated in real time.
* no real need to *UPDATE* data at the database.

This connector is not intended to be used by *projects* or software pieces that need to use enhanced SQL operations, such as the selection of a range of data, or updating operations. If it is needed to operate directly with the content of the database (such as making reports), another software pieces should be used.

This connector and the preprocessor macros give the possibility to push data to a stack, and let the connector to automatically push it to a database. The "push" operation is not done in real time, but every "x" seconds by a parallel thread, so the main *user's* thread is not interrupted.


## HOW TO USE IT

### using 1 unique database with a DAO

### using multiple DAOs with a database
