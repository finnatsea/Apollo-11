"""
AGC Block Two Self-Check
========================

A Python implementation of the Apollo Guidance Computer (AGC) self-check routines.
Based on the original AGC assembly code for Apollo 11's Lunar Module.

The self-check system verifies proper functioning of various computer components:
- Erasable memory checking
- Fixed memory checking
- Counter checking
- Cycle and shift register checking
- Rope memory checksum verification

Original assembly from: https://github.com/chrislgarry/Apollo-11
"""
import enum
import logging
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Union, Callable


class SelfCheckMode(enum.IntEnum):
    """Self-check mode options that correspond to the original SMODE values"""
    IDLE = 0           # No check, puts computer into backup idle loop 
    FULL_CHECK_1 = 1   # Everything in options 4 and 5
    FULL_CHECK_2 = 2   # Everything in options 4 and 5
    FULL_CHECK_3 = 3   # Everything in options 4 and 5
    ERASABLE_MEMORY = 4
    FIXED_MEMORY = 5
    FULL_CHECK_6 = 6   # Everything in options 4 and 5
    FULL_CHECK_7 = 7   # Everything in options 4 and 5
    FULL_CHECK_10 = 10  # Everything in options 4 and 5 (octal 10)
    CONTINUE_ON_ERROR = -0  # Same as +-10 until error is detected


# Constants from the original code
SBIT1 = 0o000001
SBIT2 = 0o000002
SBIT3 = 0o000003
SBIT4 = 0o000010
SBIT5 = 0o000020
SBIT6 = 0o000040
SBIT7 = 0o000100
SBIT8 = 0o000200
SBIT9 = 0o000400
SBIT10 = 0o001000
SBIT11 = 0o002000
SBIT12 = 0o004000
SBIT13 = 0o010000
SBIT14 = 0o020000
SBIT15 = 0o040000

S_ZERO = 0
S_1 = SBIT1
S_2 = SBIT2
S_3 = 0o000003  # THREE
S_4 = 0o000004  # FOUR
S_5 = 0o000005  # FIVE
S_6 = 0o000006  # SIX
S_7 = 0o000007  # SEVEN
S8BITS = 0o000377  # LOW8
CNTRCON = 0o000050  # Used in CNTRCHK
ERASCON1 = 0o000061  # Used in ERASCHK
ERASCON2 = 0o001373  # Used in ERASCHK
ERASCON6 = 0o001400  # Used in ERASCHK
ERASCON3 = 0o001461  # Used in ERASCHK
ERASCON4 = 0o001773  # Used in ERASCHK
S10BITS = 0o001777  # LOW10, Used in ERASCHK
SBNK03 = 0o006000  # PRIO6, Used in ROPECHK
NEG_MAXADRS = 0o037000  # HI5, For ROPECHK
SIXTY = 0o000060
SUPRCON = 0o060017  # Used in ROPECHK
S13BITS = 0o017777
CONC_S1 = 0o025252  # Used in CYCLSHFT
CONC_S2 = 0o052400  # Used in CYCLSHFT
ERASCON5 = 0o076777
S_7 = 0o077770
S_4 = -4
S_3 = -3
S_2 = -2
S_1 = -1
S_ZERO_NEG = -0


@dataclass
class AGCRegisters:
    """Simulates the AGC registers and memory locations"""
    # General registers
    a: int = 0  # Accumulator
    l: int = 0  # Lower accumulator (L)
    q: int = 0  # Q register
    ebank: int = 0  # E-Bank register
    fbank: int = 0  # Fixed bank register
    z: int = 0  # Program counter
    
    # Special registers for self-check
    cyr: int = 0  # Cycle Right register
    cyl: int = 0  # Cycle Left register
    sr: int = 0  # Shift Right register
    edop: int = 0  # Edop register (Edit Operand)
    
    # Self-check registers
    smode: int = 0  # Self-check mode
    selfret: int = 0  # Self-check return address
    sfail: int = 0  # Failure address
    scount: int = 0  # Counter
    ercount: int = 0  # Error count
    erestore: int = 0  # Restore flag
    
    # Keep registers
    skeep1: int = 0
    skeep2: int = 0
    skeep3: int = 0
    skeep4: int = 0
    skeep5: int = 0
    skeep6: int = 0
    skeep7: int = 0
    
    # Error tracking
    almcadr: int = 0  # Alarm address


class AGCMemory:
    """Simulates the AGC memory system"""
    
    def __init__(self):
        # Erasable memory (RAM)
        self.erasable = [0] * 0o2000  # 0-1777 octal
        
        # Fixed memory banks (ROM)
        self.fixed = {}  # Will hold multiple banks
        for bank in range(0o40):  # 0-37 octal banks
            self.fixed[bank] = [0] * 0o2000  # Each bank is 1024 words (0-1777 octal)
    
    def read(self, address: int) -> int:
        """Read from memory address"""
        if address < 0o2000:
            return self.erasable[address]
        else:
            # Decode bank and relative address
            bank = (address >> 10) & 0o37
            rel_addr = address & 0o1777
            return self.fixed.get(bank, [0] * 0o2000)[rel_addr]
    
    def write(self, address: int, value: int) -> None:
        """Write to memory address"""
        if address < 0o2000:
            self.erasable[address] = value & 0o177777  # 15-bit words
        else:
            # Fixed memory can't be written to in normal operation
            # This is just for simulation/testing purposes
            bank = (address >> 10) & 0o37
            rel_addr = address & 0o1777
            if bank not in self.fixed:
                self.fixed[bank] = [0] * 0o2000
            self.fixed[bank][rel_addr] = value & 0o177777


class AGCSelfCheck:
    """
    Implementation of the AGC Block Two Self-Check system
    
    This system verifies the proper functioning of various computer components
    including erasable memory, fixed memory, counters, and more.
    """
    
    def __init__(self):
        self.regs = AGCRegisters()
        self.memory = AGCMemory()
        self.logger = logging.getLogger("AGCSelfCheck")
        
        # Simulate alarm system
        self.alarm_active = False
        self.failreg = [0, 0, 0]  # FAILREG storage
        
        # For display purposes
        self.dsky_reg = {"R1": 0, "R2": 0, "R3": 0}
    
    def selfchk(self) -> None:
        """
        Main entry point for self-check routine
        
        Corresponds to the SELFCHK label in the original code
        """
        self.logger.info("Starting AGC self-check")
        self._smodechk()
    
    def _smodechk(self) -> None:
        """Check SMODE to determine self-check option to run"""
        self.regs.skeep1 = self.regs.q

        # Check for new job (simplified from original)
        self._checknj()
        
        # Check SMODE value to determine which test to run
        if self.regs.smode > 0:
            self._run_option()
        elif self.regs.smode < 0:
            self._run_option()
        else:
            # SMODE = 0, go to backup idle loop
            pass
        
        # Continue with self-check
        self.regs.scount += 1
        return
    
    def _run_option(self) -> None:
        """Run the selected self-check option"""
        option = self.regs.smode
        
        # Convert option to 0-based index
        if option < 0:
            option = -option
        
        # Subtract 7 and check if it's below zero (for options below 9)
        if option - 7 <= 0:
            self.regs.scount += 1
            option_index = option + 7 - 1  # Adjust to 0-based index
            
            # Call the appropriate option routine
            if option_index == 3:  # Option 4 (ERASCHK)
                self._eraschk()
            elif option_index == 4:  # Option 5 (ROPECHK)
                self._ropechk()
            else:
                # Other options like SOPTION1, SOPTION2, etc.
                # Just return to SKEEP1 in the original
                return
        else:
            # Illegal option, go to idle loop
            self.regs.smode = 0
            self._selfchk()  # Restart
    
    def _checknj(self) -> None:
        """
        Check for new job
        
        In the original AGC, this would check if any jobs have become active.
        For this simulation, we just save the return address.
        """
        self.regs.selfret = self.regs.q
        # Original would call ADVAN via POSTJUMP
        # We'll just return
    
    def _errors(self, q_value: int) -> None:
        """Handle error detection and response"""
        self.regs.sfail = q_value  # Save location for failure display
        self.regs.almcadr = q_value  # For display with BBANK and ERCOUNT
        self.regs.ercount += 1  # Increment error count
        
        # Trigger alarm (original would call ALARM2 with OCT 01102)
        self._alarm(0o1102)  # Self-check malfunction indicator
        
        # Check SMODE to determine response to error
        if self.regs.smode > 0:
            # Positive option, go to idle loop
            self.regs.smode = 0
            self._selfchk()
        else:
            # Negative option, continue from failure point
            return
    
    def _alarm(self, alarm_code: int) -> None:
        """Simulate AGC alarm system"""
        self.alarm_active = True
        self.failreg[0] = alarm_code
        self.logger.warning(f"ALARM: {alarm_code:o}")
    
    def _neg1chk(self, value: int) -> bool:
        """Check if value is -1"""
        # Original would use CCS A instruction
        if value > 0:
            self._errors(0)  # Return address would be in Q
            return False
        elif value == 0:
            self._errors(0)
            return False
        else:
            # The value is negative
            # Original checks if value+1 = 0 (which means value was -1)
            if value + 1 == 0:
                return True  # Value was -1
            else:
                self._errors(0)
                return False
    
    def _eraschk(self) -> None:
        """
        Erasable memory check
        
        Tests all of erasable memory by writing complement of address
        and checking if it reads back correctly.
        """
        self.logger.info("Starting erasable memory check")
        
        # Set up for first bank (non-switched erasable)
        self.regs.skeep2 = 1
        self._0ebank()
    
    def _0ebank(self) -> None:
        """Check erasable memory in bank 0"""
        self.regs.ebank = 0
        self.regs.skeep7 = ERASCON3  # 01461 - Starting address
        self.regs.skeep3 = S10BITS   # 01777 - Last address checked
        self._erasloop()
    
    def _e134567b(self) -> None:
        """Check erasable memory in banks 1,3,4,5,6,7"""
        self.regs.skeep7 = ERASCON6  # 01400 - Starting address
        self.regs.skeep3 = S10BITS   # 01777 - Last address checked
        self._erasloop()
    
    def _2ebank(self) -> None:
        """Check erasable memory in bank 2"""
        self.regs.skeep7 = ERASCON6  # 01400 - Starting address
        self.regs.skeep3 = ERASCON4  # 01773 - Last address checked
        self._erasloop()
    
    def _noebank(self) -> None:
        """Check non-switched erasable memory"""
        self.regs.skeep2 = 0
        self.regs.skeep7 = ERASCON1  # 00061 - Starting address
        self.regs.skeep3 = ERASCON2  # 01373 - Last address checked
        self._erasloop()
    
    def _erasloop(self) -> None:
        """Main loop for erasable memory checking"""
        # Preserve current E-Bank setting
        self.regs.skeep4 = self.regs.ebank
        
        # Read original values from memory
        address = self.regs.skeep7
        self.regs.skeep5 = self.memory.read(address)
        self.regs.skeep6 = self.memory.read(address + 1)
        
        # Save restore information
        self.regs.erestore = address
        
        # Write the address into itself and address+1
        self.memory.write(address, address)
        self.memory.write(address + 1, address + 1)
        
        # Check first test pattern
        check_val = (~self.memory.read(address + 1)) & 0o177777  # CS X+1
        check_val = (check_val + self.memory.read(address)) & 0o177777  # AD X
        
        if not self._neg1chk(check_val):
            # Test failed or value has been restored
            if self.regs.erestore == 0:
                # Erasable has been restored, continue to next address
                self._eloopfin()
                return
        
        # Write complement of address as second test pattern
        complement_addr = (~address) & 0o177777
        complement_addr_plus_1 = (~(address + 1)) & 0o177777
        
        self.memory.write(address, complement_addr)
        self.memory.write(address + 1, complement_addr_plus_1)
        
        # Check second test pattern
        check_val = (~self.memory.read(address)) & 0o177777  # CS X
        check_val = (check_val + self.memory.read(address + 1)) & 0o177777  # AD X+1
        
        if not self._neg1chk(check_val):
            # Test failed or value has been restored
            if self.regs.erestore == 0:
                # Erasable has been restored, continue to next address
                self._eloopfin()
                return
        
        # Restore original values
        self.memory.write(address, self.regs.skeep5)
        self.memory.write(address + 1, self.regs.skeep6)
        
        # Clear restore flag
        self.regs.erestore = 0
        
        self._eloopfin()
    
    def _eloopfin(self) -> None:
        """Finish current iteration of ERASLOOP"""
        # Check for new job
        self._checknj()
        
        # Restore E-Bank
        self.regs.ebank = self.regs.skeep4
        
        # Increment address
        self.regs.skeep7 += 1
        
        # Check if we've reached the last address
        if self.regs.skeep7 == self.regs.skeep3 + 1:
            # End of this bank
            if self.regs.skeep2 != 0:
                self._noebank()
            else:
                # Increment to next E-Bank
                self.regs.skeep2 += 1
                new_bank = self.regs.ebank + SBIT9
                self.regs.ebank = new_bank
                
                # Check for bank E2
                if new_bank + ERASCON5 == 0:
                    self._2ebank()
                elif new_bank != 0:
                    self._e134567b()
                else:
                    # End of ERASCHK
                    self.regs.ebank = ERASCON6
                    self._cntrchk()
        else:
            # Continue with next address in same bank
            self._erasloop()
    
    def _cntrchk(self) -> None:
        """
        Counter check
        
        Checks all registers from octal 60 through octal 10, including
        all counters, T6-1, cycle and shift, and all RUPT registers
        """
        self.logger.info("Starting counter check")
        self._cntrloop(CNTRCON)  # Start with counter 50 (octal)
    
    def _cntrloop(self, counter: int) -> None:
        """Main loop for counter checking"""
        self.regs.skeep2 = counter
        
        # Compute register to check (counter + 10 octal)
        reg_to_check = counter + SBIT4  # Add 10 octal
        
        # Simulate checking the counter by complementing it
        # Original would do CS 0000,INDEX A
        counter_val = self.memory.read(reg_to_check)
        
        # Check next counter if not at end
        if counter > 0:
            self._cntrloop(counter - 1)
        else:
            # Done with counter check, continue to cycle and shift check
            self._cyclshft()
    
    def _cyclshft(self) -> None:
        """
        Check cycle and shift registers
        
        Tests CYR, CYL, SR, and EDOP registers
        """
        self.logger.info("Starting cycle and shift check")
        
        # Initialize registers with test pattern
        pattern = CONC_S1  # 25252 octal
        self.regs.cyr = pattern     # Should be 12525 octal in CYR
        self.regs.cyl = pattern     # Should be 52524 octal in CYL
        self.regs.sr = pattern      # Should be 12525 octal in SR
        self.regs.edop = pattern    # Should be 00125 octal in EDOP
        
        # First check - sum the values
        check_val = self.regs.cyr + self.regs.cyl + self.regs.sr + self.regs.edop
        check_val = (check_val + CONC_S2) & 0o177777  # Add 52400 octal
        
        # Check if the result is -1
        if not self._neg1chk(check_val):
            return
        
        # Second check - another summation
        check_val = self.regs.cyr + self.regs.cyl + self.regs.sr + self.regs.edop + 1
        
        # Check if the result is -1
        if not self._neg1chk(check_val):
            return
        
        # Increment counter and continue
        self.regs.scount += 1
        self._smodechk()
    
    def _ropechk(self) -> None:
        """
        Rope memory check (ROM check)
        
        Verifies fixed memory by computing checksums of each bank
        """
        self.logger.info("Starting rope memory check")
        
        # Set SKEEP6 to -0 for ROPECHK
        self.regs.skeep6 = S_ZERO_NEG
        
        # Start with bank 0
        self.regs.skeep4 = 0  # Bank number
        self.regs.skeep7 = 1  # In common fixed bank
        
        # Initialize checksum
        self.regs.skeep1 = 0
        self.regs.skeep3 = 0  # Starting address
        self.regs.skeep5 = 1  # Counts down 2 TC SELF words
        
        # Start checking common fixed memory
        self._comadrs()
    
    def _comadrs(self) -> None:
        """Check addresses in common fixed memory"""
        # Set super bank from SKEEP4
        self.regs.l = self.regs.skeep4
        
        # Calculate memory address
        addr = (self.regs.skeep4 & 0o37000) + self.regs.skeep3
        
        # Read memory at this address
        val = self.memory.read(addr)
        
        # Add to checksum
        self._adsum(val)
        
        # Check next address
        addr += SBIT11  # Add 2000 octal
        self._adrschk(addr)
    
    def _fxfx(self) -> None:
        """Check fixed-fixed memory"""
        self.regs.skeep7 = ~0 & 0o177777  # Complement of A
        
        # Set starting address based on bank
        if self.regs.skeep7 == 0:
            self.regs.skeep3 = SBIT12  # 04000, starting address of bank 02
        else:
            self.regs.skeep3 = SBNK03  # 06000, starting address of bank 03
        
        # Initialize checksum
        self.regs.skeep1 = 0
        self.regs.skeep5 = 1  # Counts down 2 TC SELF words
        
        # Start checking fixed-fixed memory
        self._fxadrs()
    
    def _fxadrs(self) -> None:
        """Check addresses in fixed-fixed memory"""
        # Read memory at current address
        val = self.memory.read(self.regs.skeep3)
        
        # Add to checksum
        self._adsum(val)
        
        # Check next address
        self._adrschk(0)  # Value doesn't matter, handled in adrschk
    
    def _adsum(self, val: int) -> None:
        """Add value to checksum"""
        self.regs.skeep2 = val
        self.regs.skeep1 = (self.regs.skeep1 + val) & 0o177777
        self.regs.skeep1 = (0 + self.regs.skeep1) & 0o177777  # CAF S+ZERO, AD SKEEP1
        
        return (~self.regs.skeep2 + self.regs.skeep3) & 0o177777
    
    def _adrschk(self, addr: int) -> None:
        """Check if current address is the last one to be checked"""
        self.regs.l = addr
        
        # Check if at last address
        rel_addr = self.regs.skeep3 & 0o1777  # Relative address (LOW10)
        if rel_addr + NEG_MAXADRS == 0:
            # Checksum finished for this bank
            self._soption()
            return
        
        # Check if TC SELF words are finished
        if self.regs.skeep5 <= 0:
            self._soption()
            return
        
        # Check for TC SELF word
        if self.regs.l < 0:
            # This is not a TC SELF word
            self._continu()
        else:
            # This is a TC SELF word
            if self.regs.skeep5 > 0:
                self._continu(is_tc_self=True)
            else:
                # Add bugger word
                self._continu(add_bugger=True)
    
    def _continu(self, is_tc_self: bool = False, add_bugger: bool = False) -> None:
        """Continue with memory check"""
        if not is_tc_self and not add_bugger:
            # Make sure two consecutive TC SELF words are counted
            self.regs.skeep5 = 1
        
        if add_bugger:
            # Add in the bugger word
            self.regs.skeep5 = -1
        
        # Check for SHOWSUM vs ROPECHK
        if self.regs.skeep6 > 0:
            # SHOWSUM option
            # Original would check NEWJOB here
            pass
        else:
            # ROPECHK option
            self._checknj()
        
        # Move to next address
        self._adrs_plus_1()
    
    def _adrs_plus_1(self) -> None:
        """Increment address for next check"""
        self.regs.skeep3 += 1
        
        # Determine next address check based on skeep7
        if self.regs.skeep7 > 0:
            self._comadrs()
        elif self.regs.skeep7 < 0:
            self._fxadrs()
        else:
            # skeep7 = 0
            self._fxadrs()
    
    def _soption(self) -> None:
        """Option handling for ROPECHK and SHOWSUM"""
        # Get bank number
        bank_num = self.regs.skeep4 & 0o37000  # Bank bits (HI5)
        bank_num = bank_num >> 10  # Shift to get actual bank number
        
        # Check for super bank
        super_bank = self.regs.skeep4 & 0o377  # Super bank bits (S8BITS)
        
        if super_bank != 0:
            # Adjust bank number for super bank
            super_bank_num = bank_num & 7
            bank_num = super_bank_num + super_bank
        
        # Check if ROPECHK or SHOWSUM
        if self.regs.skeep6 == 0:
            # ROPECHK - continue checking
            pass
        else:
            # SHOWSUM - display results
            self._sdisplay()
            return
        
        # Make sure checksum is positive
        if self.regs.skeep1 < 0:
            self.regs.skeep1 = -self.regs.skeep1
        
        # Verify checksum equals bank number
        check_val = -bank_num + self.regs.skeep1 - 1
        
        # Check if result is -1
        if not self._neg1chk(check_val):
            return
        
        # Move to next bank
        self._nxtbnk()
    
    def _nxtbnk(self) -> None:
        """Move to next bank for checking"""
        # Check if reached last bank
        if self.regs.skeep4 == self.last_bank_to_check():
            # End of summing of banks
            self._endsums()
            return
        
        # Increment bank number
        self.regs.skeep4 += SBIT11  # 37 to 40 increments by end round carry
        
        # Check super bank
        self._chksupr()
    
    def _chksupr(self) -> None:
        """Check and handle super bank changes"""
        bank_bits = self.regs.skeep4 & 0o37000  # HI5
        
        if bank_bits == 0:
            # Increment super bank
            self._nxtsupr()
        elif bank_bits + S13BITS == 0:
            # Bank set for 30
            self.regs.skeep4 += SIXTY  # First super bank
        else:
            # Go to next bank
            self._gonxtbnk()
    
    def _nxtsupr(self) -> None:
        """Increment super bank number"""
        self.regs.skeep4 += SUPRCON  # Set bank 30 + increment super bank
        self._gonxtbnk()
    
    def _gonxtbnk(self) -> None:
        """Continue to next bank"""
        if self.regs.skeep7 > 0:
            self._commfx()
        else:
            self.regs.skeep7 = 1
            self._fxfx()
    
    def _commfx(self) -> None:
        """Set up for common fixed memory check"""
        self.regs.skeep7 = 1
        self.regs.skeep1 = 0
        self.regs.skeep3 = 0
        self.regs.skeep5 = 1  # Counts down 2 TC SELF words
        self._comadrs()
    
    def _endsums(self) -> None:
        """Handle completion of memory checksums"""
        self.logger.info("Memory check complete")
        # In original, would exit to other routines
    
    def _sdisplay(self) -> None:
        """Display sum for current bank (SHOWSUM)"""
        # Display would show:
        # R1: Bank sum (equal to bank number)
        # R2: Bank number
        # R3: Bugger word
        self.dsky_reg["R1"] = self.regs.skeep1
        self.dsky_reg["R2"] = self.regs.l  # Bank number
        self.dsky_reg["R3"] = self.regs.skeep3  # Bugger word
        
        self.logger.info(f"SHOWSUM: Bank {self.regs.l:o} sum = {self.regs.skeep1:o}")
    
    def last_bank_to_check(self) -> int:
        """Return the value of the last bank to check"""
        # In original, this would be from the LSTBNKCH constant
        # For this implementation, we'll use a fixed value
        return 0o040  # Last bank number (octal)
    
    def run_self_check(self, mode: SelfCheckMode = SelfCheckMode.FULL_CHECK_10) -> Dict:
        """
        Run the self-check system with the specified mode
        
        Args:
            mode: The self-check mode to run
            
        Returns:
            Dict with results of the self-check
        """
        self.regs.smode = int(mode)
        self.selfchk()
        
        return {
            "error_count": self.regs.ercount,
            "alarm_active": self.alarm_active,
            "failreg": self.failreg,
            "dsky": self.dsky_reg
        }
    
    def show_banksum(self) -> Dict:
        """
        Run the show-banksum function
        
        Returns:
            Dict with bank checksums
        """
        # Set up for SHOWSUM
        self.regs.skeep6 = 1  # +1 for SHOWSUM
        self._ropechk()  # This will calculate and display bank sums
        
        return {
            "current_bank": self.regs.l,
            "bank_sum": self.regs.skeep1,
            "bugger_word": self.regs.skeep3
        }


if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    
    # Create and run self-check
    agc_check = AGCSelfCheck()
    
    print("=== AGC Block Two Self-Check ===")
    print("Running erasable memory check...")
    results = agc_check.run_self_check(SelfCheckMode.ERASABLE_MEMORY)
    
    if results["error_count"] > 0:
        print(f"Check failed with {results['error_count']} errors")
    else:
        print("Check completed successfully")
