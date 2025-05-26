#include <Bela.h>
#include <include/lsl_cpp.h>
#include <vector>
#include <string>
#include <atomic>
#include <memory>
#include <fstream>
#include <sstream>
#include <include/font.h>
#include <include/linux_i2c.c>
#include <include/ssd1306.c>
#include <limits>
#include <iomanip>
#include <cmath>

// Configuration
#define USE_OLED_DISPLAY 1
const int kOledI2cDev = 1;

// Digital I/O pin configuration
const int eventStartPins[] = {0, 1, 2, 3};
const int eventEndPins[] = {12, 13, 14, 15};
const int kNumStartPins = sizeof(eventStartPins) / sizeof(eventStartPins[0]);
const int kNumEndPins = sizeof(eventEndPins) / sizeof(eventEndPins[0]);
const int kTotalPins = kNumStartPins + kNumEndPins;

// LSL Configuration
const char* streamPrefixFilter = "LSLTest";
float sampleTimeout = 0.0;

// Logging Configuration
int gElapsedTime = 0; // Placeholder for elapsed time
const int fs = 44100; // Audio sample rate, can be adjusted as needed
const size_t kEventBufferSize = 8192;  // Circular buffer size
const size_t kFlushThreshold = 1024;   // Flush when buffer reaches this size
const char* kLogFileName = "/root/timing_log.csv";

// Event structure for logging
struct TimingEvent {
    double system_timestamp;    // Bela elapsed time
    double lsl_timestamp;      // LSL timestamp or NaN for pin events
    enum Type { PIN_CHANGE, LSL_SAMPLE } type;
    
    // Pin change data
    int pin_number;
    bool pin_state;
    
    // LSL sample data
    std::string stream_name;
    std::vector<float> sample_data;
    double sample_timestamp;   // Original sample timestamp
    
    TimingEvent() : system_timestamp(0.0), lsl_timestamp(std::numeric_limits<double>::quiet_NaN()),
                   type(PIN_CHANGE), pin_number(-1), pin_state(false), sample_timestamp(0.0) {}
};

// Circular buffer for events
class EventBuffer {
private:
    std::vector<TimingEvent> buffer;
    std::atomic<size_t> write_pos{0};
    std::atomic<size_t> read_pos{0};
    std::atomic<size_t> count{0};
    
public:
    EventBuffer(size_t size) : buffer(size) {}
    
    bool push(const TimingEvent& event) {
        size_t current_count = count.load();
        if (current_count >= buffer.size()) {
            return false; // Buffer full
        }
        
        size_t pos = write_pos.load();
        buffer[pos] = event;
        write_pos.store((pos + 1) % buffer.size());
        count.fetch_add(1);
        return true;
    }
    
    bool pop(TimingEvent& event) {
        size_t current_count = count.load();
        if (current_count == 0) {
            return false; // Buffer empty
        }
        
        size_t pos = read_pos.load();
        event = buffer[pos];
        read_pos.store((pos + 1) % buffer.size());
        count.fetch_sub(1);
        return true;
    }
    
    size_t size() const { return count.load(); }
    bool empty() const { return count.load() == 0; }
};

// Global variables
std::atomic<bool> shouldResolveStreams{true};
std::vector<lsl::stream_info> availableStreams;
std::vector<lsl::stream_inlet*> streamInlets;
std::vector<std::vector<float>> streamData;
std::vector<std::string> streamNames;
bool streamsResolved = false;

// Pin state tracking
std::vector<bool> currentPinStates;
std::vector<bool> previousPinStates;

// Event logging
std::unique_ptr<EventBuffer> eventBuffer;
lsl::continuous_resolver* resolver = nullptr;
double startTime = 0.0;  // System start time for elapsed time calculation

// Auxiliary tasks
AuxiliaryTask gResolveStreamsTask;
AuxiliaryTask gPullSamplesTask;
AuxiliaryTask gLogWriterTask;

// Function declarations
void resolveStreams(void*);
void pullSamples(void*);
void writeLogData(void*);
double getElapsedTime();
void logPinChange(int pin, bool state);
void logLSLSample(const std::string& streamName, const std::vector<float>& data, 
                  double sampleTimestamp, double receiveTimestamp);
std::string formatCSVLine(const TimingEvent& event);

double getElapsedTime() {
    return gElapsedTime / fs;  // Convert to seconds
    
}

void logPinChange(int pin, bool state) {
    TimingEvent event;
    event.system_timestamp = getElapsedTime();
    event.lsl_timestamp = lsl::local_clock();
    event.type = TimingEvent::PIN_CHANGE;
    event.pin_number = pin;
    event.pin_state = state;
    
    if (!eventBuffer->push(event)) {
        rt_printf("Warning: Event buffer full, dropping pin change event\n");
    }
}

void logLSLSample(const std::string& streamName, const std::vector<float>& data, 
                  double sampleTimestamp, double receiveTimestamp) {
    TimingEvent event;
    event.system_timestamp = getElapsedTime();
    event.lsl_timestamp = receiveTimestamp;
    event.type = TimingEvent::LSL_SAMPLE;
    event.stream_name = streamName;
    event.sample_data = data;
    event.sample_timestamp = sampleTimestamp;
    
    if (!eventBuffer->push(event)) {
        rt_printf("Warning: Event buffer full, dropping LSL sample event\n");
    }
}

std::string formatCSVLine(const TimingEvent& event) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);
    
    oss << event.system_timestamp << ",";
    
    if (std::isnan(event.lsl_timestamp)) {
        oss << ",";
    } else {
        oss << event.lsl_timestamp << ",";
    }
    
    if (event.type == TimingEvent::PIN_CHANGE) {
        oss << "PIN_CHANGE," << event.pin_number << "," << (event.pin_state ? "1" : "0") << ",,,";
        
        // Add current state of all pins
        for (int i = 0; i < kTotalPins; i++) {
            int pin = (i < kNumStartPins) ? eventStartPins[i] : eventEndPins[i - kNumStartPins];
            bool state = (pin == event.pin_number) ? event.pin_state : currentPinStates[i];
            oss << (state ? "1" : "0");
            if (i < kTotalPins - 1) oss << ";";
        }
    } else {
        oss << "LSL_SAMPLE,,," << event.stream_name << "," << event.sample_timestamp << ",";
        
        // Add sample data
        for (size_t i = 0; i < event.sample_data.size(); i++) {
            oss << event.sample_data[i];
            if (i < event.sample_data.size() - 1) oss << ";";
        }
        oss << ",";
        
        // Add current state of all pins
        for (int i = 0; i < kTotalPins; i++) {
            oss << (currentPinStates[i] ? "1" : "0");
            if (i < kTotalPins - 1) oss << ";";
        }
    }
    
    oss << "\n";
    return oss.str();
}

bool setup(BelaContext *context, void *userData)
{
    startTime = 0;
    
    rt_printf("Using LSL library version: %d.%d\n", 
              lsl::library_version() / 100, 
              lsl::library_version() % 100);
    
    // Validate pin configuration
    if (kNumStartPins != kNumEndPins) {
        rt_printf("Error: Mismatched number of event start and end pins.\n");
        return false;
    }
    
    rt_printf("Setting up event pins...\n");
    
    // Initialize pin state tracking
    currentPinStates.resize(kTotalPins, false);
    previousPinStates.resize(kTotalPins, false);
    
    // Set up digital pins
    for (int i = 0; i < kNumStartPins; i++) {
        pinMode(context, 0, eventStartPins[i], INPUT);
        pinMode(context, 0, eventEndPins[i], INPUT);
        
        // Read initial states
        currentPinStates[i] = digitalRead(context, 0, eventStartPins[i]);
        currentPinStates[i + kNumStartPins] = digitalRead(context, 0, eventEndPins[i]);
        previousPinStates[i] = currentPinStates[i];
        previousPinStates[i + kNumStartPins] = currentPinStates[i + kNumStartPins];
    }
    
    // Initialize event buffer
    eventBuffer = std::make_unique<EventBuffer>(kEventBufferSize);
    
    #ifdef USE_OLED_DISPLAY
    rt_printf("Setting up OLED display...\n");
    ssd1306_init(kOledI2cDev);
    ssd1306_oled_default_config(64, 128);
    ssd1306_oled_clear_screen();
    ssd1306_oled_set_XY(0, 0);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"LSL Timing Test");
    #endif
    
    // Create auxiliary tasks
    if ((gResolveStreamsTask = Bela_createAuxiliaryTask(&resolveStreams, 50, "resolve-streams")) == 0)
        return false;
    
    if ((gPullSamplesTask = Bela_createAuxiliaryTask(&pullSamples, 80, "pull-samples")) == 0)
        return false;
    
    if ((gLogWriterTask = Bela_createAuxiliaryTask(&writeLogData, 40, "log-writer")) == 0)
        return false;
    
    // Initialize log file with CSV header
    std::ofstream logFile(kLogFileName, std::ios::out | std::ios::trunc);
    if (logFile.is_open()) {
        logFile << "system_timestamp,lsl_timestamp,event_type,pin_number,pin_state,stream_name,sample_timestamp,sample_data,pin_states\n";
        logFile.close();
        rt_printf("Created log file: %s\n", kLogFileName);
    } else {
        rt_printf("Warning: Could not create log file\n");
    }
    
    // Create continuous resolver
    resolver = new lsl::continuous_resolver();
    
    // Schedule the first resolution task
    Bela_scheduleAuxiliaryTask(gResolveStreamsTask);
    
    rt_printf("Setup complete. Monitoring %d pins.\n", kTotalPins);
    
    return true;
}

void render(BelaContext *context, void *userData)
{
    // Monitor digital pins for changes
    bool anyChange = false;
    
    // Check start pins
    for (int i = 0; i < kNumStartPins; i++) {
        bool state = digitalRead(context, 0, eventStartPins[i]);
        if (state != previousPinStates[i]) {
            currentPinStates[i] = state;
            logPinChange(eventStartPins[i], state);
            previousPinStates[i] = state;
            anyChange = true;
        }
    }
    
    // Check end pins
    for (int i = 0; i < kNumEndPins; i++) {
        bool state = digitalRead(context, 0, eventEndPins[i]);
        int pinIndex = i + kNumStartPins;
        if (state != previousPinStates[pinIndex]) {
            currentPinStates[pinIndex] = state;
            logPinChange(eventEndPins[i], state);
            previousPinStates[pinIndex] = state;
            anyChange = true;
        }
    }
    
    // Schedule auxiliary tasks
    static unsigned int count = 0;
    if (count++ % (unsigned int)(context->audioSampleRate / context->audioFrames) == 0) {
        if (shouldResolveStreams) {
            Bela_scheduleAuxiliaryTask(gResolveStreamsTask);
        }
    }
    
    if (!streamInlets.empty()) {
        Bela_scheduleAuxiliaryTask(gPullSamplesTask);
    }
    
    // Schedule log writer if buffer has enough data or periodically
    if (eventBuffer->size() >= kFlushThreshold || count % 1000 == 0) {
        Bela_scheduleAuxiliaryTask(gLogWriterTask);
    }
}

void cleanup(BelaContext *context, void *userData)
{
    // Final flush of event buffer
    Bela_scheduleAuxiliaryTask(gLogWriterTask);
    usleep(100000); // Wait 100ms for final flush
    
    // Close and clean up stream inlets
    for (auto inlet : streamInlets) {
        inlet->close_stream();
        delete inlet;
    }
    streamInlets.clear();
    
    // Clean up resolver
    delete resolver;
    
    rt_printf("Cleanup complete. Final event buffer size: %zu\n", eventBuffer->size());
}

void resolveStreams(void*)
{
    bool needToReopen = streamInlets.empty();
    availableStreams = resolver->results();
    
    if (availableStreams.empty()) {
        #ifdef USE_OLED_DISPLAY
        ssd1306_oled_clear_line(2);
        ssd1306_oled_set_XY(0, 2);
        ssd1306_oled_write_string(SSD1306_FONT_NORMAL, (char*)"No streams");
        #endif
        return;
    }
    
    if (needToReopen) {
        for (auto inlet : streamInlets) {
            inlet->close_stream();
            delete inlet;
        }
        streamInlets.clear();
        streamData.clear();
        streamNames.clear();
        
        size_t validStreams = 0;
        for (const auto& info : availableStreams) {
            if (!info.name().empty() && info.name().find(streamPrefixFilter) == 0) {
                try {
                    lsl::stream_inlet* inlet = new lsl::stream_inlet(info, 360, 0, true);
                    streamInlets.push_back(inlet);
                    streamData.push_back(std::vector<float>(info.channel_count()));
                    streamNames.push_back(info.name());
                    inlet->open_stream(1.0);
                    validStreams++;
                    
                    rt_printf("Opened stream: %s (%s), %d channels\n", 
                             info.name().c_str(), info.type().c_str(), info.channel_count());
                } catch (std::exception& e) {
                    rt_printf("Error creating inlet for %s: %s\n", info.name().c_str(), e.what());
                }
            }
        }
        
        #ifdef USE_OLED_DISPLAY
        ssd1306_oled_clear_line(2);
        ssd1306_oled_set_XY(0, 2);
        std::string streamStr = "Streams: " + std::to_string(validStreams);
        ssd1306_oled_write_string(SSD1306_FONT_NORMAL, (char*)streamStr.c_str());
        #endif
        
        streamsResolved = true;
    }
}

void pullSamples(void*)
{
    if (!streamsResolved || streamInlets.empty())
        return;
    
    double receiveTime = lsl::local_clock();
    
    for (size_t i = 0; i < streamInlets.size(); i++) {
        try {
            double sampleTimestamp = streamInlets[i]->pull_sample(streamData[i], sampleTimeout);
            
            if (sampleTimestamp != 0.0) {
                logLSLSample(streamNames[i], streamData[i], sampleTimestamp, receiveTime);
                
                rt_printf("LSL %s: [", streamNames[i].c_str());
                for (size_t j = 0; j < streamData[i].size(); j++) {
                    rt_printf("%.3f", streamData[i][j]);
                    if (j < streamData[i].size() - 1) rt_printf(", ");
                }
                rt_printf("] t=%.6f\n", sampleTimestamp);
            }
        } catch (lsl::lost_error& e) {
            rt_printf("Stream %s lost: %s\n", streamNames[i].c_str(), e.what());
            streamInlets[i]->close_stream();
            delete streamInlets[i];
            streamInlets[i] = nullptr;
            shouldResolveStreams = true;
        } catch (std::exception& e) {
            rt_printf("Error pulling sample from %s: %s\n", streamNames[i].c_str(), e.what());
        }
    }
    
    // Clean up null pointers
    auto it = streamInlets.begin();
    auto nameIt = streamNames.begin();
    auto dataIt = streamData.begin();
    
    while (it != streamInlets.end()) {
        if (*it == nullptr) {
            it = streamInlets.erase(it);
            nameIt = streamNames.erase(nameIt);
            dataIt = streamData.erase(dataIt);
        } else {
            ++it; ++nameIt; ++dataIt;
        }
    }
    
    if (streamInlets.empty()) {
        streamsResolved = false;
    }
}

void writeLogData(void*)
{
    if (eventBuffer->empty())
        return;
    
    std::ofstream logFile(kLogFileName, std::ios::out | std::ios::app);
    if (!logFile.is_open()) {
        rt_printf("Error: Could not open log file for writing\n");
        return;
    }
    
    TimingEvent event;
    size_t eventsWritten = 0;
    
    while (eventBuffer->pop(event)) {
        logFile << formatCSVLine(event);
        eventsWritten++;
        
        // Limit writes per call to avoid blocking too long
        if (eventsWritten >= 500) {
            break;
        }
    }
    
    logFile.close();
    
    if (eventsWritten > 0) {
        rt_printf("Wrote %zu events to log. Buffer size: %zu\n", eventsWritten, eventBuffer->size());
    }
}
