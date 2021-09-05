class Timer
{
public:
    Timer(void);
    void set_max_delay(unsigned long v);
    void set(void);
    boolean check(void);
    void unset(void);
private:
    unsigned long max_delay;
    unsigned long last_set;
    unsigned int bEnabled;    
};

Timer::Timer(void)
{
    max_delay = 3600000UL; // default 1 hour
}

void Timer::set_max_delay(unsigned long v)
{
    max_delay = v;
    set();
}

void Timer::set()
{
    last_set = millis();
    bEnabled=true;
	#ifdef DEBUG
	Serial.println("timer: START  ");
	#endif
}

void Timer::unset()
{
    bEnabled=false;
	#ifdef DEBUG
	Serial.println("timer: disabled  ");
	#endif
}

boolean Timer::check()
{
    unsigned long now = millis();
    if ( ( now - last_set > max_delay ) && bEnabled) {
        last_set = now;
	#ifdef DEBUG
		Serial.println("timer: jetzt!  ");
	#endif
        return true;
    }
	#ifdef DEBUG
	Serial.println("timer: warte  ");
	#endif
    return false;
}
