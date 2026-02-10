# Specialized Deployment Vectors (SDV)

# System modules
import gzip
import json
from logging.handlers import RotatingFileHandler
import os
import queue
import shutil
import threading
from pathlib import Path
from typing import Union

class GzipRotatingFileHandler(RotatingFileHandler):
    """
    Archive Compacting Vector (ACV)
    Transform the existing generic GzipRotatingFileHandler into a specialized Archive Compacting Vector (ACV) 
    that compacts obsolete archives into a more space-efficient storage unit.
    
    Features:
    - ACV works independently and automously that requires no external intervention.
    - Ingression of archives into a localized vicinity pending for compaction.
    - Utilize SWEEP Logic to scavenge for all uncompacted archives in the vicinity.
    - Apply 3-digit number coding archive tag (eg..001, .002, .003) to the archives.
    - Ejection of the compacted archives into the centralized storage unit.
    
    Workflow:
    -----------    -------------    --------------    -------------
    |         |    |           |    |            |    |           |
    | Ingress | -> | Detection | -> | Compaction | -> | Discharge |
    |         |    |           |    |            |    |           |
    -----------    -------------    --------------    -------------
    """
    
    # Deployment status of the ACV
    _ACV_buffer_queue: queue.Queue = queue.Queue(maxsize=100)   # Number of buffers allowed to be loaded into the ACV
    _ACV_is_activated: bool = False                             # Flip the switch to True when ACV is activated
    _ACV_thread: threading.Thread | None = None                 # ACV is currently dormant, waiting for archives to feed in for compact
    _ACV_lock: threading.Lock = threading.Lock()                # Prevent duplicate ACV deployment
    
    # Startup the ACV ready
    def __init__(self, *args, **kwargs) -> None:
        """
        Deploy the ACV to start running the archive compacting cycle without need of attention.

        Args:
            args: Positional arguments to be passed to the RotatingFileHandler
            kwargs: Keyword arguments to be passed to the RotatingFileHandler
        """
        super().__init__(*args, **kwargs)         # Setup a specialized initiation sequence for the ACV
        self.namer = self._apply_archive_tag      # Tag the archive with standardized number coding (eg. the .007 archive - License to kill (LTM))
        self._activate_ACV_compact_cycle()        # Get the ACV to start operation when ready

    def doRollover(self) -> None:
        """
        Override the default rollover behavior to queue the oldest obsolete archives for compression.
        """
        super().doRollover()                      # Take over the generic rollover behavior to queue the archives for compression
        
        if self.backupCount > 0:                  # Trigger the mechanism only if archive is setup
            # (TODO) Make a Github issue for the following:
            # Only trigger with .1 file, if .1 file not exist, only .2 to .999, this will not trigger.
            oldest_archive: str = self.rotation_filename(f"{self.baseFilename}.1") # Queue obsolete archive into the ACV buffer for compacting
            
            if os.path.exists(oldest_archive):    # Trigger the mechanism only if archive exists
                # (TODO) Make a Github issue for the following:
                # Currently, if queue is full, the mechanism does not do anything,
                # The remaining files not in the queue will be orphaned until next boot.
                # I think the best fix is to set bigger queue size + change the mechanism so it will detect any 3 digit number files and put in the queue when there is vacancy
                try:                              # Queue the archive 
                    GzipRotatingFileHandler._ACV_buffer_queue.put(oldest_archive, block=False) # Transfer the archive into the ACV buffer for compacting, without waiting for the queue to clear
                except queue.Full:                # If ACV buffer is full, identify with warning message
                    # (TODO) Switch to TALLE logic to report the warning
                    print(f"[TALLE] Compression queue full - dropping {oldest_archive}")   # Alert the robot system/humans the ACV buffer is full
    
    def _apply_archive_tag(self, archive_name: str) -> str:
        """
        Tag the archive pending for compacting with standardized number coding (3-digit format).
        (eg. top.secret.7 → top.secret.007)

        Arg:
            archive_name (str): Name of the archive to be tagged
        
        Return:
            str: Name of the archive with the applied tag
        """
        
        # (TODO) Add to GitHub issue: 
        # If archive_name already applied with the 3-digit tag, system still continue to apply tag forever
        archive_name_splits: list[str] = archive_name.rsplit('.', 1)             # Analyze the archive name to look for the archive number
        if len(archive_name_splits) == 2 and archive_name_splits[-1].isdigit():  # If archive number is found
            return f"{archive_name_splits[0]}.{int(archive_name_splits[1]):03d}" # Convert archive number with standardized 3-digit archive tag
        return archive_name                                                      # Otherwise, do not apply archive tag

    @classmethod
    def _activate_ACV_compact_cycle(cls) -> None:
        """
        Activate ACV to start running the archive compacting cycle without need of attention. (once only)
        """
        with cls._ACV_lock:
            if not cls._ACV_is_activated:                                       # Gatekeeping to ensure the compacting cycle only if ACV is not activated
                cls._ACV_is_activated: bool = True                              # ACV is now activated
                cls._ACV_thread: threading.Thread | None = threading.Thread(    # Set up the thread of ACV
                    target=cls._ACV_compact_cycle,                              # Give instruction to run the archive compacting cycle 
                    daemon=True,                                                # Set ACV to activate as long as the robot running, and stop when the robot is off
                    name="Archive.Compacting.Vector"                            # Name this daemon ACV
                )
                cls._ACV_thread.start()                                         # Start the thread of ACV to start compact cycle to run behind the scene
    
    @classmethod
    def _ACV_compact_cycle(cls) -> None:
        """
        ACV starts running the archive compacting cycle without need of attention.
        Operates on its own with little attention and scavenges throughout the vicinity for any archive tagged for compacting.
        """
        # (TODO) Later on, add flag to give different priority and activity level between Active Mode and Idle Mode 
        # Assign ACV to moderately low proirity level to balance stability and performance
        try:                                                                     # Assign ACV to operate with moderately low proirity level if running on Linux OS
            os.nice(10)                                                          # My archive compact agent is moderately nice, 
        except:                                                                  # Do not change priority level if OS not supported
            pass                                                                 # Go to the next step
        
        while cls._ACV_is_activated:                                             # Keep running the compact cycle as long as ACV is activated
            try:                                                                 # Attempt to run the compact cycle to see if any issue
                # Wait for the queued archive in the ACV buffer to arrive
                loaded_archive: str | None = cls._ACV_buffer_queue.get(timeout=1.0) # Load the archive from the ACV buffer to compactor and free up one queue spot
                
                if loaded_archive is None:                                       # Terminate the compact cycle if there is no more queued archive
                    break                                                        # Terminate the compact cycle
                
                archive_location: Path = Path(loaded_archive).parent             # Retrieve the location of the queued archive
                
                # SWEEP LOGIC: Scavenges ALL 3-digit numbered archives in vicinity
                # This catches orphaned archives that could not be loaded into the ACV buffer due to overflow
                for archive in archive_location.glob("*.[0-9][0-9][0-9]"):       # Scavenge the location of the archive to locate any tagged archive pending for compacting
                    cls._compact_archive(archive)                                # Compact the archive into smaller size packet and disintegrate the archive
                
                cls._ACV_buffer_queue.task_done()                                # Indicate compact operation is completed , cross out one task in the todo list
                
            except queue.Empty:                                                  # If the ACV buffer is empty, move on
                continue                                                         # Continue the cycle without doing anything
            except Exception as e:                                               # If any other issue detected, identify with warning message
                # (TODO) Switch to TALLE logic to report the warning
                # (TODO) Separate to different error level
                print(f"[TALLE] Compression worker error: {e}")                  # Alert the robot system/humans that something wrong during the compact cycle
    
    @classmethod
    def _compact_archive(cls, loaded_archive: Union[str, Path]) -> None:
        """
        Compact a single archive onto a more space-efficient storage unit one by one.
        
        Uses temporary archive to ensure atomic operation, ie. either compact operation succeeds completely 
        or original archive remains intact.

        Args:
            loaded_archive (Union[str, Path]): Path to the archive to be compacted
        """
        archive_location: Path = Path(loaded_archive)                             # Double insure the archive location is actually a location
        
        # Prevent the ACV from compacting the same archive more than once
        if archive_location.suffix == ".gz" or not archive_location.exists():     # No need to compact if archive is already compacted or magically vanished
            return                                                                # Next!
        
        compacted_archive_location = archive_location.with_suffix(archive_location.suffix + '.gz')   # Set up the permanent storage of the compacted archive
        tmp_archive_location = compacted_archive_location.with_suffix('.gz.tmp')                     # Set up the temporary storage of the compacted archive for validation before moving over to permanent storage
        
        try:                                                                              # Try to compact the archive to see if any issue
            # Compact archive to a temporary location
            with open(archive_location, 'rb') as infeed:                                  # Load the archive into the compactor infeed for compacting
                with gzip.open(tmp_archive_location, 'wb', compresslevel=6) as outfeed:   # Compact the archive and put onto the compactor outfeed for validation
                    shutil.copyfileobj(infeed, outfeed)
            
            # Validate if the compacting process is successful
            if tmp_archive_location.exists() and tmp_archive_location.stat().st_size > 0:  # If temporary archive exists is created and contains data
                tmp_archive_location.rename(compacted_archive_location)                    # Move the compacted archive to the permanent storage
                archive_location.unlink()                                                  # Disintegrate the original archive as the compacted archive is already safely stored
            else:
                raise OSError("Failed to compress archive")                                # Raise an error if temporary archive is not created or contains no data
        
        except Exception as e:                                                    # If any other issue detected, identify with warning message
            # (TODO) Switch to TALLE logic to report the warning
            # (TODO) Separate to different error level
            print(f"[TALLE] Failed to compress {archive_location}: {e}")          # Alert the robot system/humans that something wrong during the compact cycle
            
            if tmp_archive_location.exists():                                     # If temporary archive exists, disintegrate the temporary file
                tmp_archive_location.unlink()                                     # Disintegrate the temporary archive
    
    @classmethod
    def shutdown(cls) -> None:
        """
        Shutdown the ACV and stop the archive compacting cycle gracefully.
        ACV will be shutdown only during robot shutdown to ensure all queued archives are being compacted.
        """
        if cls._ACV_is_activated:                               # Trigger the shutdown sequence only if ACV is activated
            cls._ACV_is_activated: bool = False                 # Flip the switch to False to indicate ACV is deactivated
            
            # Shutdown the ACV gracefully
            try:                                                # Attempt to shutdown the ACV without waiting for the queue to clear
                cls._ACV_buffer_queue.put(None, timeout=2.0)    # Transfer the shutdown signal into the ACV buffer, giving 2 seconds for the buffer to clear one slot
            except queue.Full:                                  # If the ACV buffer is full, move on
                pass                                            # Move on to the next step
            
            # Wait for ACV buffer to drain
            cls._ACV_buffer_queue.join()                        # Wait for all queued archives in the ACV buffer to be compacted before shutting down
            
            # Wait for ACV thread
            if cls._ACV_thread:                                 # If ACV thread is in operation
                cls._ACV_thread.join(timeout=5.0)               # Wait for the ACV thread to finish before shutting down
                if cls._ACV_thread.is_alive():                  # If ACV thread is still running after 5 seconds
                    # (TODO) Switch to TALLE logic to report the warning
                    print("[TALLE] ACV not able to shutdown. Cutting power forcefully...") # Alert the robot system/humans that ACV failed to shutdown gracefully
