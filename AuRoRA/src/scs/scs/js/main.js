// Event listeners
        userInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') sendSignal();
        });
        
        imagePrompt.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') sendSignal();
        });
        
        // Initial state
        setEmotion('happy');
        
        // Test functions
        window.testEmotion = function(emotion) {
            const testEmojis = {
                'happy': '😊',
                'sad': '😢',
                'angry': '😠',
                'surprised': '😮',
                'thinking': '🤔',
                'loving': '❤️',
                'petted': '💖'
            };
            setEmotion(emotion, testEmojis[emotion]);
        };
        
        window.testPet = function() {
            const fakeEvent = { clientX: null, clientY: null };
            handlePet(fakeEvent);
        };
        
        window.toggleWebSearch = toggleWebSearch;
        window.handleNegativeReward = handleNegativeReward;