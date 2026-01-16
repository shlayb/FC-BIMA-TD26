/*
    Program to store vehicle modes
    
    how to change modes:
    MODE = mode_name -> mode()

    *change mode_name with name of the modes

*/

class Mode {
    public:
        virtual const char *mode() const = 0;
};

class MANU : public Mode {
    public:
        const char *mode() const override {return"MANU";}
};

class FBWA : public Mode {
    public:
        const char *mode() const override {return"FBWA";}
};

class AUTO : public Mode {
    public:
        const char *mode() const override {return"AUTO";}
};

class ALTH : public Mode {
    public:
        const char *mode() const override {return"ALTH";}
};

class POSH : public Mode {
    public:
        const char *mode() const override {return"POSH";}
};

class TKOF : public Mode {
    public:
        const char *mode() const override {return"TKOF";}
};

class CRUS : public Mode {
    public:
        const char *mode() const override {return"CRUS";}
};

class LAND : public Mode {
    public:
        const char *mode() const override {return"LAND";}
};

class COPT : public Mode {
    public:
        const char *mode() const override {return"COPT";}
};

class TRNS : public Mode {
    public:
        const char *mode() const override {return"TRNS";}
};

class FIXD : public Mode {
    public:
        const char *mode() const override {return"FIXD";}
};

class INIT : public Mode {
    public:
        const char *mode() const override {return"INIT";}
};

class STBL : public Mode {
    public:
        const char *mode() const override {return"STBL";}
};

class LOIT : public Mode {
    public:
        const char *mode() const override {return"LOIT";}
};

Mode* manual = new MANU;
Mode* fbwa = new FBWA;
Mode* automatic = new AUTO;
Mode* althold = new ALTH;
Mode* poshold = new POSH;
Mode* takeoff = new TKOF;
Mode* cruise = new CRUS;
Mode* land = new LAND;
Mode* copter = new COPT;
Mode* transition = new TRNS;
Mode* fixed_wing = new FIXD;
Mode* initMODE = new INIT;
// Mode* stblize = new STBL;
// Mode* loitr = new LOIT;


const char *MODE;




