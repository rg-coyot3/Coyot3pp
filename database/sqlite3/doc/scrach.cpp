#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <sqlite3.h>
#include <chrono>
#include <string>

struct Packet {
    int64_t id;
    std::string name;
    std::string description;
};

class DatabaseComponent {
public:
    DatabaseComponent() : stop_thread(false) {
        // Start the thread that processes packets
        db_thread = std::thread(&DatabaseComponent::processPackets, this);
    }

    ~DatabaseComponent() {
        // Gracefully stop the thread when the object is destroyed
        stop();
    }

    // Set the database location
    void set_database_location(const std::string& dblocation) {
        std::lock_guard<std::mutex> lock(mutex);
        db_location = dblocation;
        openDatabase();
    }

    // Set the table name
    void set_table_name(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex);
        table_name = name;
    }

    // Push a packet into the vector for processing
    void push(const Packet& packet) {
        std::lock_guard<std::mutex> lock(mutex);
        packet_queue.push_back(packet);
        cv.notify_one();  // Notify the processing thread
    }

private:
    void processPackets() {
        while (!stop_thread) {
            std::this_thread::sleep_for(std::chrono::seconds(1));  // Check every second

            std::vector<Packet> packetsToProcess;
            {
                std::lock_guard<std::mutex> lock(mutex);
                packetsToProcess.swap(packet_queue);  // Swap to avoid locking while processing
            }

            // If there are packets to insert, insert them into the database
            if (!packetsToProcess.empty()) {
                insertPacketsBatch(packetsToProcess);
            }
        }
    }

    void openDatabase() {
        if (db) {
            sqlite3_close(db);  // Close any existing database connection
        }
        int rc = sqlite3_open(db_location.c_str(), &db);
        if (rc) {
            std::cerr << "Can't open database: " << sqlite3_errmsg(db) << std::endl;
            return;
        }

        // Create the table if it doesn't exist
        std::string createTableSQL = "CREATE TABLE IF NOT EXISTS " + table_name + " ("
            "id INTEGER PRIMARY KEY, "
            "name TEXT, "
            "description TEXT);";

        char* errMsg = nullptr;
        rc = sqlite3_exec(db, createTableSQL.c_str(), nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK) {
            std::cerr << "SQL error: " << errMsg << std::endl;
            sqlite3_free(errMsg);
        }
    }

    void insertPacketsBatch(const std::vector<Packet>& packets) {
        // Begin a transaction
        const std::string beginTransactionSQL = "BEGIN TRANSACTION;";
        sqlite3_exec(db, beginTransactionSQL.c_str(), nullptr, nullptr, nullptr);

        // Prepare the SQL statement for batch insert
        std::string sql = "INSERT INTO " + table_name + " (id, name, description) VALUES (?, ?, ?);";
        sqlite3_stmt* stmt = nullptr;

        int rc = sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(db) << std::endl;
            return;
        }

        // Bind values for each packet and execute the statement
        for (const auto& packet : packets) {
            sqlite3_bind_int64(stmt, 1, packet.id);
            sqlite3_bind_text(stmt, 2, packet.name.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_text(stmt, 3, packet.description.c_str(), -1, SQLITE_STATIC);

            rc = sqlite3_step(stmt);
            if (rc != SQLITE_DONE) {
                std::cerr << "Execution failed: " << sqlite3_errmsg(db) << std::endl;
            }

            // Reset the statement so we can reuse it
            sqlite3_reset(stmt);
        }

        // Commit the transaction
        const std::string commitTransactionSQL = "COMMIT;";
        sqlite3_exec(db, commitTransactionSQL.c_str(), nullptr, nullptr, nullptr);

        // Finalize the statement
        sqlite3_finalize(stmt);
    }

    void stop() {
        stop_thread = true;
        cv.notify_one();  // Wake up the thread to exit if it's waiting
        if (db_thread.joinable()) {
            db_thread.join();  // Wait for the thread to finish
        }
        if (db) {
            sqlite3_close(db);  // Close the database connection when done
        }
    }

    std::string db_location;
    std::string table_name = "packets";  // Default table name
    sqlite3* db = nullptr;
    std::vector<Packet> packet_queue;
    std::thread db_thread;
    std::mutex mutex;
    std::condition_variable cv;
    bool stop_thread;
};

int main() {
    // Create a DatabaseComponent object
    DatabaseComponent dbComponent;

    // Set the database location and table name
    dbComponent.set_database_location("packets.db");
    dbComponent.set_table_name("packet_data");

    // Create some sample packets
    Packet packet1 = {1, "Packet1", "This is the first packet"};
    Packet packet2 = {2, "Packet2", "This is the second packet"};
    Packet packet3 = {3, "Packet3", "This is the third packet"};

    // Push packets into the DatabaseComponent
    dbComponent.push(packet1);
    dbComponent.push(packet2);
    dbComponent.push(packet3);

    // Wait for a while to let the thread process the packets
    std::this_thread::sleep_for(std::chrono::seconds(3));

    return 0;
}




using namespace std;

static int callback(void* data, int argc, char** argv, char** azColName)
{
    int i;
    fprintf(stderr, "%s: ", (const char*)data);

    for (i = 0; i < argc; i++) {
        printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
    }

    printf("\n");
    return 0;
}

int main(int argc, char** argv)
{
    sqlite3* DB;
    int exit = 0;
    exit = sqlite3_open("example.db", &DB);
    string data("CALLBACK FUNCTION");

    string sql("SELECT * FROM PERSON;");
    if (exit) {
        std::cerr << "Error open DB " << sqlite3_errmsg(DB) << std::endl;
        return (-1);
    }
    else
        std::cout << "Opened Database Successfully!" << std::endl;

    int rc = sqlite3_exec(DB, sql.c_str(), callback, (void*)data.c_str(), NULL);

    if (rc != SQLITE_OK)
        cerr << "Error SELECT" << endl;
    else {
        cout << "Operation OK!" << endl;
    }

    sqlite3_close(DB);
    return (0);
}





///
sqlite3_stmt *stmt;
const char *sql = "SELECT ID, Name FROM User";
int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
if (rc != SQLITE_OK) {
    print("error: ", sqlite3_errmsg(db));
    return;
}
while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
    int id           = sqlite3_column_int (stmt, 0);
    const char *name = sqlite3_column_text(stmt, 1);
    // ...
}
if (rc != SQLITE_DONE) {
    print("error: ", sqlite3_errmsg(db));
}
sqlite3_finalize(stmt);
///
