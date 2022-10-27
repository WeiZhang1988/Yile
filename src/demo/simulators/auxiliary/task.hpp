class Task {
public:
    //default constructor
    virtual void run() {
        //do something here
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout<<"hello"<<std::endl;
    }
};