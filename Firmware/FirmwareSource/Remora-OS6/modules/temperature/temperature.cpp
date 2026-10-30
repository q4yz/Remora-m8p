#include "temperature.h"


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

void createTemperature()
{
    printf("Make Temperature measurement object\n");
    const char* comment = module["Comment"];
    printf("%s\n",comment);

    int pv = module["PV[i]"];
    const char* sensor = module["Sensor"];

    ptrProcessVariable[pv]  = &txData.processVariable[pv];

    if (!strcmp(sensor, "Thermistor"))
    {
        const char* pinSensor = module["Thermistor"]["Pin"];
        float beta =  module["Thermistor"]["beta"];
        int r0 = module["Thermistor"]["r0"];
        int t0 = module["Thermistor"]["t0"];

        // slow module with 1 hz update
        int updateHz = 1;
        Module* temperature = new Temperature(*ptrProcessVariable[pv], PRU_SERVOFREQ, updateHz, sensor, pinSensor, beta, r0, t0);
        servoThread->registerModule(temperature);
    }
}

/***********************************************************************
*                METHOD DEFINITIONS                                    *
************************************************************************/

Temperature::Temperature(volatile float &ptrFeedback, int32_t threadFreq, int32_t slowUpdateFreq, std::string sensorType, std::string pinSensor, float beta, int r0, int t0) :
  Module(threadFreq, slowUpdateFreq),
  ptrFeedback(&ptrFeedback),
  sensorType(sensorType),
  pinSensor(pinSensor),
  sensorError(false),
	beta(beta),
	r0(r0),
	t0(t0)
{
    if (this->sensorType == "Thermistor")
    {
        printf("Creating Thermistor Tempearture measurement @ pin %s\n", this->pinSensor.c_str());
        //cout <<"Creating Thermistor Tempearture measurement @ pin " << this->pinSensor << endl;
        this->Sensor = new Thermistor(this->pinSensor, this->beta, this->r0, this->t0);
    }
    // TODO: Add more sensor types as needed

    // Take some readings to get the ADC up and running before moving on
    this->slowUpdate();
    this->slowUpdate();
    printf("Start temperature = %f\n", this->temperaturePV);
    //cout << "Start temperature = " << this->temperaturePV << endl;
}

void Temperature::update()
{
  return;
}

void Temperature::slowUpdate()
{
	this->temperaturePV = this->Sensor->getTemperature();

    // check for disconnected temperature sensor
    // slowUpdate() runs in the servo thread interrupt: a printf there blocks it for
    // milliseconds (115200 baud) and the board misses SPI frames, so print changes only
    if (this->temperaturePV > 0)
    {
        if (this->sensorError) printf("Temperature sensor on pin %s OK again\n", this->pinSensor.c_str());
        this->sensorError = false;
        *(this->ptrFeedback) = this->temperaturePV;
    }
    else
    {
        if (!this->sensorError) printf("Temperature sensor error, pin %s reading = %f\n", this->pinSensor.c_str(), this->temperaturePV);
        this->sensorError = true;
        *(this->ptrFeedback) = 999;
    }

}
