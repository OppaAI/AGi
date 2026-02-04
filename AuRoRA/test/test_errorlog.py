if __name__ == "__main__":
    from scs.igniter_temp import LoggerProxy
    import os
    
    # 1. Force the log directory to be local for this test
    test_logger = LoggerProxy("VCS.TEST", log_dir="./test_logs")
    
    # 2. Try to log something
    print("--- SPENCER'S DIAGNOSTIC CHECK ---")
    test_logger.info("If you can see this in the console, the logger is ALIVE!")
    test_logger.error("This should be in errors.log and robot.log")
    
    # 3. Check if the files actually exist
    log_file = os.path.join(".", "test_logs", "robot.log")
    if os.path.exists(log_file):
        print(f"SUCCESS: Log file created at {os.path.abspath(log_file)}")
    else:
        print("FAILURE: No log file found. Check your folder permissions, meatbag!")