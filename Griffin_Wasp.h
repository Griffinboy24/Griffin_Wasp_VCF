#pragma once
#include <JuceHeader.h>
#include <cmath>

namespace project {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    using namespace juce;
    using namespace hise;
    using namespace scriptnode;

    namespace FunctionsClasses {
        // Additional helper functions or classes can be defined here if needed.
    }

    template <int NV>
    struct Griffin_Wasp : public data::base
    {
        SNEX_NODE(Griffin_Wasp);

        struct MetadataClass
        {
            SN_NODE_ID("Griffin_Wasp");
        };

        //==============================================================================
        // Node Properties 
        //==============================================================================
        static constexpr bool isModNode() { return false; }
        static constexpr bool isPolyphonic() { return NV > 1; }
        static constexpr bool hasTail() { return false; }
        static constexpr bool isSuspendedOnSilence() { return false; }
        static constexpr int getFixChannelAmount() { return 2; }

        static constexpr int NumDisplayBuffers = 0;
        static constexpr int NumTables = 0;
        static constexpr int NumSliderPacks = 0;
        static constexpr int NumAudioFiles = 0;
        static constexpr int NumFilters = 0;

        // Parameter indices:
        // 0: Cutoff, 1: Resonance, 2: FilterType (0=Lowpass, 1=Bandpass, 2=Highpass)
        int filterType = 0; // Default: Lowpass

        // Structure for the three filter outputs.
        struct SVFOutput {
            float hp, bp, lp;
        };

        //==============================================================================
        // Audio Effect Class (TPT State Variable Filter with 3 outputs)
        //==============================================================================
        class AudioEffect
        {
        public:
            AudioEffect(float initCutoff = 5000.0f, float initResonance = 0.5f)
                : cutoff(initCutoff), resonance(initResonance)
            {
                sampleRate = 44100.0;
                resetState();
                recalcCoeffs();
            }

            void prepare(double sr)
            {
                sampleRate = sr;
                resetState();
                recalcCoeffs();
            }

            // Process one sample and return HP, BP, and LP outputs.
            SVFOutput processSample(float x)
            {
                // TPT SVF algorithm:
                float hp = (x - (state_lp + r * state_bp)) * a1;
                state_bp += g * hp;
                state_lp += g * state_bp;
                SVFOutput out{ hp, state_bp, state_lp };
                return out;
            }

            void updateCutoff(float newCutoff)
            {
                cutoff = newCutoff;
                recalcCoeffs();
            }

            void updateResonance(float newRes)
            {
                resonance = newRes;
                recalcCoeffs();
            }

            void resetState()
            {
                state_bp = 0.0f;
                state_lp = 0.0f;
            }

        private:
            double sampleRate;
            float cutoff;     // Filter cutoff (Hz)
            float resonance;  // Resonance control (0 to 1)

            // SVF state variables
            float state_bp = 0.0f;
            float state_lp = 0.0f;

            // Coefficients for the TPT SVF
            float g = 0.0f;   // Integration coefficient from tan(pi*cutoff/sampleRate)
            float r = 0.0f;   // Damping factor (related to Q)
            float a1 = 0.0f;  // Normalization factor

            // Recalculate coefficients using the bilinear transform.
            void recalcCoeffs()
            {
                // Compute an offset as in the original design.
                float offset = 1.696e-5f * cutoff;
                // Compute Q from resonance and offset.
                float Q = 1.0f / (resonance + offset);
                // In a TPT SVF, the damping factor r = 1/(2*Q)
                r = 1.0f / (2.0f * Q);
                // g is computed via the tan pre-warping.
                g = std::tan(M_PI * cutoff / static_cast<float>(sampleRate));
                // Normalization factor.
                a1 = 1.0f / (1.0f + g);
            }
        };

        // Two instances for stereo processing.
        AudioEffect leftChannelEffect;
        AudioEffect rightChannelEffect;

        //==============================================================================
        // Main Processing Functions
        //==============================================================================
        void prepare(PrepareSpecs specs)
        {
            float sr = specs.sampleRate;
            leftChannelEffect.prepare(sr);
            rightChannelEffect.prepare(sr);
        }

        void reset() {}

        // Process block: for each channel, process each sample through the SVF.
        // The main output is determined by the FilterType parameter,
        // and the two alternate responses are sent to display buffers.
        template <typename ProcessDataType>
        void process(ProcessDataType& data)
        {
            auto& fixData = data.template as<ProcessData<getFixChannelAmount()>>();
            auto audioBlock = fixData.toAudioBlock();
            int blockSize = data.getNumSamples();

            float* leftChannelData = audioBlock.getChannelPointer(0);
            float* rightChannelData = audioBlock.getChannelPointer(1);

           

            for (int i = 0; i < blockSize; ++i)
            {
                auto outL = leftChannelEffect.processSample(leftChannelData[i]);
                auto outR = rightChannelEffect.processSample(rightChannelData[i]);

                float mainL = (filterType == 0) ? outL.lp : (filterType == 1 ? outL.bp : outL.hp);
                float mainR = (filterType == 0) ? outR.lp : (filterType == 1 ? outR.bp : outR.hp);

                leftChannelData[i] = mainL;
                rightChannelData[i] = mainR;

            
            }
        }

        //==============================================================================
        // Parameter Handling
        //==============================================================================
        // Parameter index 0: Cutoff; 1: Resonance; 2: FilterType (0=LP, 1=BP, 2=HP)
        template <int P>
        void setParameter(double v)
        {
            if (P == 0)
            {
                leftChannelEffect.updateCutoff(static_cast<float>(v));
                rightChannelEffect.updateCutoff(static_cast<float>(v));
            }
            else if (P == 1)
            {
                leftChannelEffect.updateResonance(static_cast<float>(v));
                rightChannelEffect.updateResonance(static_cast<float>(v));
            }
            else if (P == 2)
            {
                filterType = static_cast<int>(v);
            }
        }

        // Create GUI parameters.
        // Note: FilterType valid values: 0=Lowpass, 1=Bandpass, 2=Highpass.
        void createParameters(ParameterDataList& data)
        {
            {
                parameter::data p("Cutoff", { 20.0, 20000.0, 0.01 });
                registerCallback<0>(p);
                p.setDefaultValue(5000.0);
                data.add(std::move(p));
            }
            {
                parameter::data p("Resonance", { 0.0, 1.0, 0.01 });
                registerCallback<1>(p);
                p.setDefaultValue(0.5);
                data.add(std::move(p));
            }
            {
                parameter::data p("FilterType", { 0, 2, 1 });
                registerCallback<2>(p);
                p.setDefaultValue(0);
                data.add(std::move(p));
            }
        }

        //==============================================================================
        // External Data Handling
        //==============================================================================
        void setExternalData(const ExternalData& ed, int index)
        {
            // Example external data handling.
        }

        //==============================================================================
        // Event Handling (MIDI or other events)
        //==============================================================================
        void handleHiseEvent(HiseEvent& e)
        {
            // Example event handling.
        }

        //==============================================================================
        // Frame Processing (required but not used in block processing)
        //==============================================================================
        template <typename FrameDataType>
        void processFrame(FrameDataType& data) {}
    };

} // namespace project
