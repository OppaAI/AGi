// Send message (with web search support)
        function sendSignal() {
            const val = userInput.value.trim();
            
            if (isStreaming) return;
            
            // Reset RL tracking for new user message
            latestResponseId = null;
            positiveRewardCount = 0;
            negativeRewardCount = 0;
            negCounter.classList.remove('show');
            
            // Check if image is attached
            if (currentImage) {
                const promptText = imagePrompt.value.trim() || "What's in this image?";
                
                // Display in chat
                createMessageElement('user', `📷 ${promptText}`);
                
                if (isROSConnected && window.imageInputTopic) {
                    // Send to CNS Bridge
                    const imageData = {
                        prompt: promptText,
                        image: currentImage,
                        web_search: webSearchEnabled
                    };
                    
                    window.imageInputTopic.publish(new ROSLIB.Message({ 
                        data: JSON.stringify(imageData) 
                    }));
                    
                    setEmotion('thinking', '🤔');
                } else {
                    // Offline mode
                    setTimeout(() => {
                        const offlineResponse = "😔: I can't process images in offline mode, but nice picture! 📸";
                        detectAndSetEmotion(offlineResponse);
                        createMessageElement('grace', offlineResponse);
                    }, 500);
                }
                
                // Clear image
                removeImage();
                createFloatingHeart();
                
            } else if (val) {
                // Normal text message
                createMessageElement('user', val);
                
                if (isROSConnected && window.cnsTopic) {
                    // FIXED: Send plain text only
                    // Python auto-detects web search via should_search()
                    window.cnsTopic.publish(new ROSLIB.Message({ 
                        data: val  // Plain text, not JSON
                    }));
                    
                    console.log('💬 Sent:', val);
                    console.log('🌐 Web search:', webSearchEnabled ? 'enabled (visual only)' : 'disabled');
                } else {
                    // Offline mode
                    setTimeout(() => {
                        const offlineResponse = "💖: I'm running in offline mode, but I still love you! Pet me instead? 💕";
                        detectAndSetEmotion(offlineResponse);
                        createMessageElement('grace', offlineResponse);
                    }, 500);
                }
                
                setEmotion('surprised', '✨');
                setTimeout(() => setEmotion('happy'), 500);
                
                userInput.value = '';
                createFloatingHeart();
            }
        }
        
        function createFloatingHeart() {
            const heart = document.createElement('div');
            heart.className = 'float-hearts';
            heart.innerHTML = ['💖', '💕', '💗', '💝', '💘'][Math.floor(Math.random() * 5)];
            heart.style.left = Math.random() * window.innerWidth + 'px';
            heart.style.animationDuration = (3 + Math.random() * 2) + 's';
            document.body.appendChild(heart);
            
            setTimeout(() => heart.remove(), 5000);
        }
        
        