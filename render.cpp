#include <Bela.h>
#include <include/lsl_cpp.h>
#include <vector>
#include <string>
#include <atomic>
#include <include/font.h>
#include <include/linux_i2c.c>
#include <include/ssd1306.c>

// Configuration

#define USE_OLED_DISPLAY 1 // Set to 1 to enable OLED display, 0 to disable
const int kOledI2cDev = 1; // I2C device number for OLED display (0,1,2) for /dev/i2c-0, /dev/i2c-1, /dev/i2c-2

// Digital I/O pin configuration

// Event start pins (multiple e.g., FSR, tirgger input, etc.)
const int eventStartPins[] = {0, 1, 2, 3}; // Example pins for event start
// corresponding event end pins (e.g. photodiode, etc.)
const int eventEndPins[] = {12, 13, 14, 15}; // Example pins for event end

// LSL

const char* streamPrefixFilter = "LSLTest"; // Prefix to filter LSL streams

float sampleTimeout = 0.0; // 0.0 for non-blocking

std::atomic<bool> shouldResolveStreams{true};


// YOU PROBABLY DON'T NEED TO CHANGE ANYTHING BELOW THIS LINE

// LSL stream handling
std::vector<lsl::stream_info> availableStreams;
std::vector<lsl::stream_inlet*> streamInlets;
bool streamsResolved = false;


// Continuous stream resolver for background discovery
lsl::continuous_resolver* resolver = nullptr;

// Buffer for storing stream data
std::vector<std::vector<float>> streamData;
std::vector<std::string> streamNames;

// Auxiliary tasks
AuxiliaryTask gResolveStreamsTask;
AuxiliaryTask gPullSamplesTask;

// Function declarations
void resolveStreams(void*);
void pullSamples(void*);

// digital pin state ring-buffer



bool setup(BelaContext *context, void *userData)
{
    // Print LSL library version
    rt_printf("Using LSL library version: %d.%d\n", 
              lsl::library_version() / 100, 
              lsl::library_version() % 100);
    // test that there are an equal number of start and end pins
    if(sizeof(eventStartPins) != sizeof(eventEndPins)) {
        rt_printf("Error: Mismatched number of event start and end pins.\n");
        return false;
    }
    rt_printf("Setting up event pins...\n");
    // set up all digital pins as inputs
    for(int i = 0; i < sizeof(eventStartPins); i++) {
        pinMode(context, 0, eventStartPins[i], INPUT);
        pinMode(context, 0, eventEndPins[i], INPUT);
    }

    #ifdef USE_OLED_DISPLAY
        rt_printf("Setting up OLED display...\n");
        ssd1306_init(kOledI2cDev);
        ssd1306_oled_default_config(64, 128);
        ssd1306_oled_clear_screen();

        ssd1306_oled_set_XY(0, 0);
        // strncpy(line, "Hello Bela!", sizeof(line));
        ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"Found Streams:");
    #endif // USE_OLED_DISPLAY
    // Create auxiliary tasks
    if ((gResolveStreamsTask = Bela_createAuxiliaryTask(&resolveStreams, 50, "resolve-streams")) == 0)
        return false;
    
    if ((gPullSamplesTask = Bela_createAuxiliaryTask(&pullSamples, 80, "pull-samples")) == 0)
        return false;
    
    // Create continuous resolver
    resolver = new lsl::continuous_resolver();

    
    // Schedule the first resolution task
    Bela_scheduleAuxiliaryTask(gResolveStreamsTask);
    
    return true;
}

void render(BelaContext *context, void *userData)
{
    // Schedule stream resolution periodically (every ~1 second)
    static unsigned int count = 0;
    if(count++ % (unsigned int)(context->audioSampleRate / context->audioFrames) == 0) {
        if(shouldResolveStreams) {
            Bela_scheduleAuxiliaryTask(gResolveStreamsTask);
        }
    }
    
    // If we have active streams, schedule sample pulling for every render cycle
    if(!streamInlets.empty()) {
        Bela_scheduleAuxiliaryTask(gPullSamplesTask);
    }
}

void cleanup(BelaContext *context, void *userData)
{
    // Close and clean up stream inlets
    for(auto inlet : streamInlets) {
        inlet->close_stream();
        delete inlet;
    }
    streamInlets.clear();
    
    // Clean up resolver
    delete resolver;
}

// Function to resolve available LSL streams
void resolveStreams(void*)
{
    // Check if any inlets are open
    bool needToReopen = streamInlets.empty();
    
    // Get results from the continuous resolver
    availableStreams = resolver->results();
    
    if(availableStreams.empty()) {
        #ifdef USE_OLED_DISPLAY
            ssd1306_oled_clear_line(2);
            ssd1306_oled_set_XY(10, 2);
            ssd1306_oled_write_string(SSD1306_FONT_NORMAL, (char*)"-> None");
        #endif // USE_OLED_DISPLAY
        rt_printf("No LSL streams found\n");
        return;
    }
    
    if(needToReopen) {
        
        // Clean up existing inlets (shouldn't be any if needToReopen is true)
        for(auto inlet : streamInlets) {
            inlet->close_stream();
            delete inlet;
        }
        streamInlets.clear();
        streamData.clear();
        streamNames.clear();
        
        // Create inlets for each stream
        for(size_t i = 0; i < availableStreams.size(); i++) {
            const auto& info = availableStreams[i];
            rt_printf("  Stream %zu: %s (%s), %d channels\n", 
                     i, info.name().c_str(), info.type().c_str(), info.channel_count());
            // match stream prefix
            if(!info.name().empty() && info.name().find(streamPrefixFilter) != 0) {
                rt_printf("  Skipping stream %s due to prefix filter\n", info.name().c_str());
                continue;
            }
            // Create a new inlet
            try {
                // Create inlet with a longer buffer and recovery option
                lsl::stream_inlet* inlet = new lsl::stream_inlet(info, 360, 0, true);
                streamInlets.push_back(inlet);
                
                // Prepare vector for data
                streamData.push_back(std::vector<float>(info.channel_count()));
                streamNames.push_back(info.name());
                
                // Open the stream
                inlet->open_stream(1.0); // 1.0 second timeout
                rt_printf("  Stream opened successfully\n");
            } catch(std::exception& e) {
                rt_printf("  Error creating inlet: %s\n", e.what());
            }
        }

        rt_printf("Found %zu LSL streams:\n", streamInlets.size());
        #ifdef USE_OLED_DISPLAY
        // Display on OLED
            ssd1306_oled_clear_line(2);
            ssd1306_oled_set_XY(10, 2);
            std::string streamCountStr = "-> " + std::to_string(streamInlets.size()) + " / " + std::to_string(availableStreams.size());
            ssd1306_oled_write_string(SSD1306_FONT_NORMAL, (char*)streamCountStr.c_str());
        #endif // USE_OLED_DISPLAY
        
        
        
        streamsResolved = true;
    }
}

// Function to pull samples from active streams
void pullSamples(void*)
{
    if(!streamsResolved || streamInlets.empty())
        return;
    
    for(size_t i = 0; i < streamInlets.size(); i++) {
        try {
            double timestamp = streamInlets[i]->pull_sample(streamData[i], sampleTimeout);
            
            if(timestamp != 0.0) {
                // Print stream data
                rt_printf("%s: [", streamNames[i].c_str());
                for(size_t j = 0; j < streamData[i].size(); j++) {
                    rt_printf("%f", streamData[i][j]);
                    if(j < streamData[i].size() - 1)
                        rt_printf(", ");
                }
                rt_printf("] (t=%f)\n", timestamp);
            }
        } catch(lsl::lost_error& e) {
            rt_printf("Stream %s lost: %s\n", streamNames[i].c_str(), e.what());
            
            // Close stream and mark for reopening
            streamInlets[i]->close_stream();
            delete streamInlets[i];
            streamInlets[i] = nullptr;
            
            // Trigger stream resolution on next cycle
            shouldResolveStreams = true;
        } catch(std::exception& e) {
            rt_printf("Error pulling sample from %s: %s\n", streamNames[i].c_str(), e.what());
        }
    }
    
    // Clean up any closed streams
    auto it = streamInlets.begin();
    auto nameIt = streamNames.begin();
    auto dataIt = streamData.begin();
    
    while(it != streamInlets.end()) {
        if(*it == nullptr) {
            it = streamInlets.erase(it);
            nameIt = streamNames.erase(nameIt);
            dataIt = streamData.erase(dataIt);
        } else {
            ++it;
            ++nameIt;
            ++dataIt;
        }
    }
    
    // If all streams were lost, set flag to resolve again
    if(streamInlets.empty()) {
        streamsResolved = false;
        rt_printf("All streams lost, will try to resolve again\n");
    }
}
