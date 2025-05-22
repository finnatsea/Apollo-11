/**
 * AGC Block Two Self-Check
 * ========================
 *
 * A TypeScript implementation of the Apollo Guidance Computer (AGC) self-check routines.
 * Based on the original AGC assembly code for Apollo 11's Lunar Module.
 *
 * The self-check system verifies proper functioning of various computer components:
 * - Erasable memory checking
 * - Fixed memory checking
 * - Counter checking
 * - Cycle and shift register checking
 * - Rope memory checksum verification
 *
 * Original assembly from: https://github.com/chrislgarry/Apollo-11
 */

export enum SelfCheckMode {
    /** No check, puts computer into backup idle loop */
    IDLE = 0,
    /** Everything in options 4 and 5 */
    FULL_CHECK_1 = 1,
    /** Everything in options 4 and 5 */
    FULL_CHECK_2 = 2,
    /** Everything in options 4 and 5 */
    FULL_CHECK_3 = 3,
    ERASABLE_MEMORY = 4,
    FIXED_MEMORY = 5,
    /** Everything in options 4 and 5 */
    FULL_CHECK_6 = 6,
    /** Everything in options 4 and 5 */
    FULL_CHECK_7 = 7,
    /** Everything in options 4 and 5 (octal 10) */
    FULL_CHECK_10 = 8, // Octal 10 is decimal 8
    /** Same as +-10 until error is detected */
    CONTINUE_ON_ERROR = -0, // Special case for -0
}

// Constants from the original code
const SBIT1 = 0o000001;
const SBIT2 = 0o000002;
// const SBIT3 = 0o000003; // THREE in AGC, not directly used as SBIT3 in Python
const SBIT4 = 0o000010;
// const SBIT5 = 0o000020;
// const SBIT6 = 0o000040;
const SBIT7 = 0o000100;
// const SBIT8 = 0o000200;
const SBIT9 = 0o000400;
const SBIT10 = 0o001000;
const SBIT11 = 0o002000;
const SBIT12 = 0o004000;
// const SBIT13 = 0o010000;
// const SBIT14 = 0o020000;
const SBIT15 = 0o040000;

const S_ZERO = 0;
// const S_1 = SBIT1; // Not used directly in Python class methods
// const S_2 = SBIT2;
// const S_3 = 0o000003;  // THREE
// const S_4 = 0o000004;  // FOUR
// const S_5 = 0o000005;  // FIVE
// const S_6 = 0o000006;  // SIX
// const S_7 = 0o000007;  // SEVEN
// const S8BITS = 0o000377;  // LOW8
const CNTRCON = 0o000050;  // Used in counterCheck
const ERASCON1 = 0o000061;  // Used in erasableMemoryCheck
const ERASCON2 = 0o001373;  // Used in erasableMemoryCheck
const ERASCON6 = 0o001400;  // Used in erasableMemoryCheck
const ERASCON3 = 0o001461;  // Used in erasableMemoryCheck
const ERASCON4 = 0o001773;  // Used in erasableMemoryCheck
const S10BITS = 0o001777;  // LOW10, Used in erasableMemoryCheck
const SBNK03 = 0o006000;  // PRIO6, Used in ropeMemoryCheck
const NEG_MAXADRS_CONSTANT = 0o76000; // -02000 in 15-bit 1's complement (HI5 in AGC is 076000)
const SIXTY = 0o000060;
const SUPRCON = 0o060017;  // Used in ropeMemoryCheck
const S13BITS = 0o017777;
const CONC_S1 = 0o025252;  // Used in cycleAndShiftCheck
const CONC_S2 = 0o052400;  // Used in cycleAndShiftCheck
const ERASCON5 = 0o076777;
// const S_NEG_7 = 0o077770; // S-7 in AGC
// const S_NEG_4 = -4;
// const S_NEG_3 = -3;
// const S_NEG_2 = -2;
// const S_NEG_1 = -1;
const S_ZERO_NEG = -0; // Used for ROPECHK SKEEP6

// Mask for 15-bit AGC data words if 1's complement interpretation
const AGC_WORD_MASK = 0o77777;
// Mask used in Python example (possibly 16-bit unsigned representation)
const PY_WORD_MASK = 0o177777;


interface DskyRegisters {
    R1: number;
    R2: number;
    R3: number;
}

export class AGCRegisters {
    // General registers
    a: number = 0;  // Accumulator
    l: number = 0;  // Lower accumulator (L)
    q: number = 0;  // Q register
    ebank: number = 0;  // E-Bank register
    // fbank: number = 0;  // Fixed bank register (not explicitly in python example's AGCRegisters)
    // z: number = 0;  // Program counter (implicit)

    // Special registers for self-check (as per AGC code names)
    cyr: number = 0;  // Cycle Right register
    cyl: number = 0;  // Cycle Left register
    sr: number = 0;  // Shift Right register (also used for Super Bank in SOPTION)
    edop: number = 0;  // Edit Operand register (EDOP)

    // Self-check state registers
    smode: number = 0;  // Self-check mode (SMODE)
    selfRet: number = 0;  // Self-check return address (SELFRET)
    sFail: number = 0;  // Failure address (SFAIL)
    sCount: number = 0;  // Counter (SCOUNT)
    erCount: number = 0;  // Error count (ERCOUNT)
    eRestore: number = 0;  // Restore flag/address for erasable check (ERESTORE)

    // SKEEP registers (temporary storage for self-check routines)
    sKeep1: number = 0;
    sKeep2: number = 0;
    sKeep3: number = 0;
    sKeep4: number = 0;
    sKeep5: number = 0;
    sKeep6: number = 0;
    sKeep7: number = 0;

    // Error tracking
    almcadr: number = 0;  // Alarm address (ALMCADR)

    constructor() {
        this.reset();
    }

    reset(): void {
        this.a = 0;
        this.l = 0;
        this.q = 0;
        this.ebank = 0;
        this.cyr = 0;
        this.cyl = 0;
        this.sr = 0;
        this.edop = 0;
        this.smode = 0;
        this.selfRet = 0;
        this.sFail = 0;
        this.sCount = 0;
        this.erCount = 0;
        this.eRestore = 0;
        this.sKeep1 = 0;
        this.sKeep2 = 0;
        this.sKeep3 = 0;
        this.sKeep4 = 0;
        this.sKeep5 = 0;
        this.sKeep6 = 0;
        this.sKeep7 = 0;
        this.almcadr = 0;
    }
}

export class AGCMemory {
    // Erasable memory (RAM) - 1024 words (0-01777 octal)
    public erasable: number[] = new Array(0o2000).fill(0);

    // Fixed memory banks (ROM) - 32 banks (0-37 octal), each 1024 words
    public fixed: Record<number, number[]> = {};

    constructor() {
        for (let bank = 0; bank < 0o40; bank++) {
            this.fixed[bank] = new Array(0o2000).fill(0);
        }
    }

    // Apply 15-bit mask for AGC data words (1's complement)
    // The Python example often uses 0o177777 (16-bit), here we use PY_WORD_MASK for consistency.
    private applyMask(value: number): number {
        return value & PY_WORD_MASK;
    }

    read(address: number): number {
        if (address < 0o2000) { // Erasable memory addresses
            return this.applyMask(this.erasable[address]);
        } else { // Fixed memory addresses
            const bank = (address >> 10) & 0o37; // Extract bank number (0-37 octal)
            const relAddr = address & 0o1777;    // Extract relative address within bank (0-1777 octal)
            return this.applyMask(this.fixed[bank]?.[relAddr] ?? 0);
        }
    }

    write(address: number, value: number): void {
        const maskedValue = this.applyMask(value);
        if (address < 0o2000) {
            this.erasable[address] = maskedValue;
        } else {
            // Fixed memory is ROM, but allow writes for simulation/initialization
            const bank = (address >> 10) & 0o37;
            const relAddr = address & 0o1777;
            if (!this.fixed[bank]) {
                this.fixed[bank] = new Array(0o2000).fill(0);
            }
            this.fixed[bank][relAddr] = maskedValue;
        }
    }
}

export class AGCSelfCheck {
    private regs = new AGCRegisters();
    private memory = new AGCMemory(); // For testing, we'd populate this

    public alarmActive = false;
    public failReg: [number, number, number] = [0, 0, 0];
    public dskyReg: DskyRegisters = { R1: 0, R2: 0, R3: 0 };

    constructor(initialMemory?: AGCMemory, initialRegisters?: AGCRegisters) {
        if (initialMemory) this.memory = initialMemory;
        if (initialRegisters) this.regs = initialRegisters;
    }
    
    // Corresponds to SELFCHK label
    public selfCheckEntry(): void {
        console.info("Starting AGC self-check");
        this.sModeCheck();
    }

    // Corresponds to SMODECHK
    private sModeCheck(): void {
        this.regs.sKeep1 = this.regs.q; // Original: QXCH SKEEP1 (QXCH implies exchange, TC CHEKNJ uses SKEEP1)
                                      // Python: self.regs.skeep1 = self.regs.q

        this.checkNewJob(); // Simplified from original: TC CHEKNJ

        const currentSmode = this.regs.smode;

        if (currentSmode > 0o10 || (currentSmode < 0 && currentSmode !== S_ZERO_NEG) || 
            (currentSmode > SelfCheckMode.FULL_CHECK_10 && currentSmode !== S_ZERO_NEG && currentSmode > 0) ) { // Octal 10 is decimal 8
            // Illegal option (SMODE > OCT 10 or unhandled negative) -> go to idle loop
            // Original: BNKOPTN TC SIDLOOP
            this.regs.smode = SelfCheckMode.IDLE; // Put into idle loop
            console.warn(`Illegal SMODE ${currentSmode.toString(8)}, going to IDLE.`);
            this.selfCheckEntry(); // Re-enter self-check, which will likely idle
            return;
        }
        
        if (currentSmode === SelfCheckMode.IDLE) { // +0 case
            // No check, put computer into backup idle loop
            console.info("SMODE is +0, entering backup idle loop (simulated).");
            return; // End of check for +0
        }

        if (currentSmode === SelfCheckMode.CONTINUE_ON_ERROR) { // -0 case
             // Handled by error recovery logic: continue with self-check.
             // Fall through to SOPTIONS or error handling will manage this.
        }
        
        this.runSelectedOption();

        // Original: INCR SCOUNT; TC SKEEP1 (Continue with self-check from SKEEP1)
        // Python: self.regs.scount += 1; return (implicitly continues)
        // Here, if an option ran, it either completed or set up for continuation.
        // The main loop (if any) would call sModeCheck again.
        // For now, we increment sCount. A real AGC loop isn't modeled here.
        this.regs.sCount = (this.regs.sCount + 1) & PY_WORD_MASK;
        
        // If smode is not IDLE and not an error halt, it implies continuation.
        // A full executive loop is not modeled here.
        // If an option leads to SIDLOOP, it sets SMODE to 0 and calls SELFCHK.
        // If an option leads to error handling, error handling decides next step.
    }
    
    // Corresponds to SOPTIONS logic
    private runSelectedOption(): void {
        let option = this.regs.smode;
        
        // Original AGC SOPTIONS logic: AD S-7; BZMF +2; ... INDEX A; TC SOPTIONx
        // S-7 is 077770. AD S-7 means A = A + 077770.
        // This is complex. Python simplifies it. Let's use Python's direct mapping.
        let effectiveOption = option;
        if (option < 0 && option !== S_ZERO_NEG) {
            effectiveOption = -option; // Use absolute value for negative options
        } else if (option === S_ZERO_NEG) { // -0 option
            effectiveOption = SelfCheckMode.FULL_CHECK_10; // Behaves like +-10 until error
        }

        // Based on AGC structure and Python interpretation for options 1-7, 10(octal)
        switch (effectiveOption) {
            case SelfCheckMode.FULL_CHECK_1: // opt 1: TC SKEEP1 (was TC+TCF) -> All
            case SelfCheckMode.FULL_CHECK_2: // opt 2: TC SKEEP1 (was IN:OUT1) -> All
            case SelfCheckMode.FULL_CHECK_3: // opt 3: TC SKEEP1 (was COUNTCHK) -> All
            case SelfCheckMode.FULL_CHECK_6: // opt 6: TC SKEEP1 -> All
            case SelfCheckMode.FULL_CHECK_7: // opt 7: TC SKEEP1 -> All
            case SelfCheckMode.FULL_CHECK_10: // opt 10 (octal 8 decimal): TC SKEEP1 -> All
                console.info(`Running FULL CHECK (ERASABLE_MEMORY and FIXED_MEMORY) for option ${option.toString(8)}.`);
                this.erasableMemoryCheck();
                if (this.alarmActive && this.regs.smode > 0) return; // Error and positive option = halt
                this.ropeMemoryCheck();
                break;
            case SelfCheckMode.ERASABLE_MEMORY: // opt 4: TC ERASCHK
                console.info(`Running ERASABLE_MEMORY check for option ${option.toString(8)}.`);
                this.erasableMemoryCheck();
                break;
            case SelfCheckMode.FIXED_MEMORY: // opt 5: TC ROPECHK
                console.info(`Running FIXED_MEMORY check for option ${option.toString(8)}.`);
                this.ropeMemoryCheck();
                break;
            default:
                // BNKOPTN -> TC SIDLOOP
                console.warn(`Invalid SMODE option ${option.toString(8)} mapped to ${effectiveOption}. Going to IDLE.`);
                this.regs.smode = SelfCheckMode.IDLE;
                this.selfCheckEntry(); // Effectively TC SELFCHK
                return;
        }
    }

    // Corresponds to CHEKNJ
    private checkNewJob(): void {
        // Original: QXCH SELFRET; TC POSTJUMP; CADR ADVAN
        // Simplified: Save return address (Q) into selfRet.
        // The actual job check isn't simulated.
        this.regs.selfRet = this.regs.q;
    }

    // Corresponds to PRERRORS and ERRORS
    private handleError(qValueFromCaller?: number): void {
        // qValueFromCaller would be the address in Q at the time of TC -1CHK or similar.
        // Original ERRORS starts: INHINT; CA Q; TS SFAIL; TS ALMCADR; INCR ERCOUNT
        // The qValue is the "address+1 of where error was detected" if error came from -1CHK
        // For simplicity, we'll use a generic marker or the qValue if provided.
        const errorLocation = qValueFromCaller ?? this.regs.q; // regs.q might be program counter

        this.regs.sFail = errorLocation;
        this.regs.almcadr = errorLocation;
        this.regs.erCount = (this.regs.erCount + 1) & PY_WORD_MASK;

        this.triggerAlarm(0o1102); // Self-check malfunction indicator

        // Original: CCS SMODE determines next step
        // SIDLOOP (CA S+ZERO; TS SMODE; TC SELFCHK) -> if SMODE was positive
        // TC SFAIL (TC Q) -> if SMODE was negative (continue with self-check)
        // -0 OPTION (TC SFAIL) -> proceeds from line after error
        if (this.regs.smode > 0) {
            console.warn(`Error detected with SMODE ${this.regs.smode.toString(8)}. Halting check and going to IDLE.`);
            this.regs.smode = SelfCheckMode.IDLE;
            this.selfCheckEntry(); // TC SELFCHK -> effectively restart/idle
        } else { // smode <= 0 (includes -0)
            console.warn(`Error detected with SMODE ${this.regs.smode.toString(8)}. Attempting to continue check.`);
            // For negative options or -0, continue. The specific continuation point (TC SFAIL)
            // would depend on what Q (SFAIL) holds. In this simulation, the calling
            // function might just proceed or the main loop continues sModeCheck.
            // If -0, it implies continuing execution flow where error was detected.
            // This is hard to model without direct execution.
            // The Python code generally returns or continues the loop.
        }
    }

    // Corresponds to TCALARM2
    private triggerAlarm(alarmCode: number): void {
        this.alarmActive = true;
        this.failReg[0] = alarmCode & PY_WORD_MASK; // FAILREG SET (actual regs are FAILREG, FAILREG +1, FAILREG +2)
        console.warn(`ALARM TRIGGERED: Code 0${alarmCode.toString(8)}`);
        // Display info (simulated)
        console.info(`DSKY Info (on V05N09E): FAILREG[0]=0${this.failReg[0].toString(8)}`);
        console.info(`DSKY Info (on V05N08E): R1(ErrorAddr+1)=0${this.regs.almcadr.toString(8)}, R2(BBCHF)=BB.SELFCHK, R3(ErrCount)=${this.regs.erCount}`);
    }

    // Corresponds to -1CHK logic: check if A contains -1 (077777 in 15-bit 1's complement)
    // Returns true if A is -1, false otherwise (and calls handleError)
    private checkIfNegativeOne(valueInA: number): boolean {
        // AGC: CCS A; TCF PRERRORS (if A > 0); TCF PRERRORS (if A == 0); (A < 0 path follows)
        //      CCS A; TCF PRERRORS (if A < -1); TC Q (if A == -1, Q holds return)
        const maskedValue = valueInA & PY_WORD_MASK; // Ensure consistent masking

        if (maskedValue === (PY_WORD_MASK)) { // -1 in 1's complement (all bits set) or 2's complement
            return true;
        } else {
             // Pass the value that was in A, as Q would effectively be the next instruction.
            this.handleError(this.regs.q); // regs.q is PC, so effectively error at current location
            return false;
        }
    }

    // ERASCHK: Erasable Memory Check
    public erasableMemoryCheck(): void {
        console.info("Starting erasable memory check (ERASCHK)");
        this.regs.sKeep2 = 1; // Controls non-switchable ERAM check with bank numbers in EBANK
        this.checkErasableBank0(); // Start with 0EBANK
        
        // After all erasable checks, original ERASCHK would flow into CNTRCHK.
        // Here we assume erasableMemoryCheck completes its scope.
        if (!this.alarmActive || this.regs.smode <= 0) {
             console.info("Erasable memory check finished.");
        }
    }

    // 0EBANK: Check erasable memory in bank 0
    private checkErasableBank0(): void {
        this.regs.ebank = 0;
        this.regs.sKeep7 = ERASCON3; // 01461 - Starting address for this segment
        this.regs.sKeep3 = S10BITS;  // 01777 - Last address for this segment
        this.erasableMemoryLoop();
    }
    
    // E134567B: Check erasable memory in banks 1,3,4,5,6,7 (uses sKeep7=01400, sKeep3=01777)
    private checkErasableBanksSwitched(): void { // Covers E134567B like logic
        this.regs.sKeep7 = ERASCON6; // 01400
        this.regs.sKeep3 = S10BITS;  // 01777
        this.erasableMemoryLoop();
    }

    // 2EBANK: Check erasable memory in bank 2 (uses sKeep7=01400, sKeep3=01773)
    private checkErasableBank2Switched(): void {
        this.regs.sKeep7 = ERASCON6; // 01400
        this.regs.sKeep3 = ERASCON4; // 01773
        this.erasableMemoryLoop();
    }
    
    // NOEBANK: Check non-switched erasable (sKeep2 = +0)
    private checkNonSwitchedErasable(): void {
        this.regs.sKeep2 = 0; // Mark as non-switched check
        this.regs.sKeep7 = ERASCON1; // 00061 - Starting address
        this.regs.sKeep3 = ERASCON2; // 01373 - Last address
        this.erasableMemoryLoop();
    }

    // ERASLOOP: Main loop for erasable memory checking
    private erasableMemoryLoop(): void {
        while (true) {
            if (this.alarmActive && this.regs.smode > 0) return; // Halt on error if positive SMODE

            this.regs.sKeep4 = this.regs.ebank; // Store C(EBANK)

            const addrX = this.regs.sKeep7;
            const addrX1 = (addrX + 1) & PY_WORD_MASK;

            // Store C(X) and C(X+1) in sKeep5 and sKeep6
            // Original: NDX SKEEP7; DCA 0000; DXCH SKEEP5 (complicated indirect ops)
            // Python: self.regs.skeep5 = self.memory.read(address); self.regs.skeep6 = self.memory.read(address + 1)
            this.regs.sKeep5 = this.memory.read(addrX); // B(X)
            // Assuming SKEEP6 was intended for B(X+1) from python example logic for DXCH SKEEP5
            // The DXCH SKEEP5 in AGC is: SKEEP5 <-> MEM[L], L <-> A. If L was addrX, A was value from addrX+1.
            // Simpler: read both directly.
            const valAtAddrX1 = this.memory.read(addrX1);
            this.regs.sKeep6 = valAtAddrX1; // Store B(X+1)

            this.regs.eRestore = addrX; // If restart, restore C(X) and C(X+1)

            // Write own address in X and X+1
            this.memory.write(addrX, addrX);
            this.memory.write(addrX1, addrX1); // Original puts X in X, X+1 in X+1. Python does too.
                                            // AGC: NDX A; DXCH 0000 (A had addrX+1, L had addrX)

            // Test 1: CS (X+1), AD (X) -> should be -1
            // A = -(X+1) + X = -1
            let testVal = ( ( (~this.memory.read(addrX1)) & PY_WORD_MASK) + this.memory.read(addrX) ) & PY_WORD_MASK;
            if (!this.checkIfNegativeOne(testVal)) {
                if (this.regs.eRestore === 0) { // Erasable was restored by interrupt logic?
                    this.finishErasableLoopIteration(); // YES, EXIT ERASLOOP (for this address)
                    if (this.regs.sKeep7 > this.regs.sKeep3) break; // Check if loop should terminate
                    continue;
                }
                 // Error occurred and not restored, checkIfNegativeOne already called handleError
                if (this.alarmActive && this.regs.smode > 0) return; // Halt
            }
            if (this.regs.eRestore === 0) { // Check again if restored during checkIfNegativeOne
                 this.finishErasableLoopIteration();
                 if (this.regs.sKeep7 > this.regs.sKeep3) break;
                 continue;
            }


            // Test 2: Write complement of address, then CS (X), AD (X+1) -> should be -1
            // A = -(~X) + ~(X+1) ? This seems off.
            // AGC: DCS 0000 (X), DXCH 0000 (X+1) -> Store ~AddrX in AddrX, ~AddrX1 in AddrX1
            //      CS 0000 (X), AD 0001 (X+1)
            //      A = -(~AddrX) + ~(AddrX+1) -> if AddrX=5, AddrX+1=6. A = -(~5) + ~6. This is complex.
            // Python: complement_addr = (~address) & PY_WORD_MASK; ...
            //         check_val = (~self.memory.read(address)) & PY_WORD_MASK  # CS X (which holds ~address)
            //         check_val = (check_val + self.memory.read(address + 1)) & PY_WORD_MASK # AD X+1 (which holds ~(address+1))
            //         So, A = -(~AddrX) + ~(AddrX+1). If AddrX=N, A = -(-N-1) + (-N-1-1) = N+1 -N-2 = -1. This is correct.

            this.memory.write(addrX, (~addrX) & PY_WORD_MASK);
            this.memory.write(addrX1, (~addrX1) & PY_WORD_MASK);

            testVal = ( ( (~this.memory.read(addrX)) & PY_WORD_MASK) + this.memory.read(addrX1) ) & PY_WORD_MASK;
            if (!this.checkIfNegativeOne(testVal)) {
                 if (this.regs.eRestore === 0) {
                    this.finishErasableLoopIteration();
                    if (this.regs.sKeep7 > this.regs.sKeep3) break;
                    continue;
                }
                if (this.alarmActive && this.regs.smode > 0) return;
            }
             if (this.regs.eRestore === 0) {
                 this.finishErasableLoopIteration();
                 if (this.regs.sKeep7 > this.regs.sKeep3) break;
                 continue;
            }

            // Restore original contents B(X) and B(X+1)
            // Original: DCA SKEEP5; NDX SKEEP7; DXCH 0000
            this.memory.write(addrX, this.regs.sKeep5);
            this.memory.write(addrX1, this.regs.sKeep6);

            this.regs.eRestore = 0; // Clear restore flag

            this.finishErasableLoopIteration();
            if (this.regs.sKeep7 > this.regs.sKeep3) { // Check if current segment is done
                break; 
            }
        }
        // After loop for a segment finishes (sKeep7 > sKeep3)
        this.transitionToNextErasableSegment();
    }

    // ELOOPFIN logic
    private finishErasableLoopIteration(): void {
        // RELINT (not simulated)
        this.checkNewJob();
        this.regs.ebank = this.regs.sKeep4; // Restore C(EBANK)
        this.regs.sKeep7 = (this.regs.sKeep7 + 1) & PY_WORD_MASK; // Increment current address being checked
                                                              // +1 then +1 again in AGC (INCR, CS, AD, BZF+2, TC ERASLOOP)
                                                              // Python simplified this to a single increment and loop condition.
                                                              // AGC: INCR SKEEP7; CS SKEEP7; AD SKEEP3; BZF +2 (means SKEEP3 - SKEEP7 == 0)
                                                              // So loop while SKEEP7 <= SKEEP3. My loop condition is SKEEP7 > SKEEP3.
    }
    
    private transitionToNextErasableSegment(): void {
        // This logic is from the end of ERASLOOP / ELOOPFIN in AGC before looping or exiting ERASCHK
        // CCS SKEEP2; TC NOEBANK (+0 in SKEEP2)
        // INCR SKEEP2; CA EBANK; AD SBIT9; TS EBANK (EBANK = EBANK+0400)
        // AD ERASCON5 (76777); BZF 2EBANK (if EBANK+0400+76777 == 0 -> EBANK was E1, now E2)
        // CCS EBANK; TC E134567B
        // CA ERASCON6; TS EBANK (End of ERASCHK)
        
        if (this.regs.sKeep2 !== 0) { // SKEEP2 was initially 1 (for 0EBANK)
            // This means we finished a switched bank check or the initial 0EBANK.
            // Try to go to NOEBANK if this was the path from 0EBANK.
            // The logic is a bit convoluted. Python's state transitions are:
            // _0ebank -> _erasloop -> _eloopfin -> if end of bank: _noebank() OR _2ebank() OR _e134567b() OR _cntrchk()
            // Let's follow a simplified state progression based on sKeep2 and ebank.

            // Based on Python structure which is clearer:
            // _0ebank calls _erasloop. _eloopfin advances. If bank ends:
            //   if skeep2 != 0 (was 1 from _0ebank), call _noebank (which sets skeep2=0)
            //   else (skeep2 was 0 from _noebank), increment skeep2.
            //     new_bank = ebank + SBIT9.
            //     if new_bank + ERASCON5 == 0 -> _2ebank
            //     elif new_bank != 0 -> _e134567b
            //     else -> end of ERASCHK (go to _cntrchk)

            // This current function is called when a segment (defined by sKeep7 to sKeep3) is done.
            // Logic from AGC's ELOOPFIN when (CS SKEEP7, AD SKEEP3, BZF +2) branches to next block:
            if (this.regs.sKeep2 !== 0) { // Was checking bank 0 (0EBANK) or a switched bank from E134567B/2EBANK
                 // This logic is complex because of how AGC structures this.
                 // For now, let's assume one pass through the different segments.
                 // A more robust state machine would be needed for perfect replication.
                 // If we just finished 0EBANK (sKeep2=1, ebank=0)
                 if (this.regs.ebank === 0 && this.regs.sKeep2 === 1) {
                    this.checkNonSwitchedErasable(); // NOEBANK
                 } 
                 // If we finished NOEBANK (sKeep2=0 was set by it, then _eloopfin increments it to 1)
                 // This part of the flow is tricky to map directly without full AGC state.
                 // The Python flow has distinct functions that call _erasloop.
                 // Let's simplify: after 0EBANK, assume we go to NOEBANK, then to a representative switched bank.
                 // This is not a perfect match of the intricate AGC flow.
                 else if (this.regs.ebank === 0 && this.regs.sKeep2 === 0) { // Placeholder for after NOEBANK
                    this.regs.ebank = (this.regs.ebank + SBIT9) & PY_WORD_MASK; // e.g. to Bank 1 (0400)
                    this.checkErasableBanksSwitched(); // E134567B
                 } else if (this.regs.ebank === (SBIT9 & PY_WORD_MASK) ) { // After bank 1 type check
                    this.regs.ebank = (this.regs.ebank + SBIT9) & PY_WORD_MASK; // e.g. to Bank 2 (1000)
                    this.checkErasableBank2Switched(); // 2EBANK
                 } else {
                    // End of all planned erasable checks
                    // Original AGC: CA ERASCON6; TS EBANK then flows to CNTRCHK
                    this.regs.ebank = ERASCON6; // Set EBANK to a final value per AGC
                    this.counterCheck();
                 }
            } else { // SKEEP2 was 0 (coming from NOEBANK path)
                this.regs.sKeep2 = (this.regs.sKeep2 + 1) & PY_WORD_MASK; // now SKEEP2 = 1
                
                const newBankCandidate = (this.regs.ebank + SBIT9) & PY_WORD_MASK;
                this.regs.ebank = newBankCandidate;

                if (((newBankCandidate + ERASCON5) & PY_WORD_MASK) === 0) { // Check for Bank E2
                    this.checkErasableBank2Switched(); // 2EBANK
                } else if (newBankCandidate !== 0) { // Check for EBANKS 1,3,4,5,6,7
                    this.checkErasableBanksSwitched(); // E134567B
                } else { // End of ERASCHK
                    this.regs.ebank = ERASCON6; // As per AGC
                    this.counterCheck(); // Proceed to next major check
                }
            }


        } else {
             // This path means SKEEP2 was 0, so we were in NOEBANK logic
             // INCR SKEEP2 -> SKEEP2 becomes 1
             this.regs.sKeep2 = (this.regs.sKeep2 + 1) & PY_WORD_MASK;
             this.regs.ebank = (this.regs.ebank + SBIT9) & PY_WORD_MASK; // EBANK = EBANK + 0400
             
             // AD ERASCON5 (76777), BZF 2EBANK
             if (((this.regs.ebank + ERASCON5) & PY_WORD_MASK) === 0) {
                 this.checkErasableBank2Switched(); // 2EBANK
             } 
             // CCS EBANK, TC E134567B (if EBANK != 0 after incr and not path to 2EBANK)
             else if (this.regs.ebank !==0) {
                 this.checkErasableBanksSwitched(); // E134567B
             }
             // else (EBANK became 0 after +SBIT9, but didn't satisfy ERASCON5 check) -> END ERASCHK
             else {
                 this.regs.ebank = ERASCON6; // Finalize EBANK state
                 this.counterCheck(); // Proceed to next set of checks
             }
        }
    }
    
    // CNTRCHK: Counter Check
    public counterCheck(): void {
        if (this.alarmActive && this.regs.smode > 0) return;
        console.info("Starting counter check (CNTRCHK)");
        
        // Original: CA CNTRCON (050); CNTRLOOP: TS SKEEP2; AD SBIT4 (+10 octal); INDEX A; CS 0000; ...
        // This checks registers from OCTAL 10 through OCTAL 60 (inclusive? 50+10=60)
        // For simulation, this is a placeholder as direct register access like INDEX A; CS 0000 is complex.
        // Python code also has a simplified loop that doesn't do much.
        let counterAddr = CNTRCON; // Starts at 050 octal
        this.regs.sKeep2 = counterAddr; 

        while(this.regs.sKeep2 >= 0o10) { // Loop from 050 down to 010 (or 000 if it goes further)
                                     // Original AGC logic seems to be: check M[X+010] for X from 050 down to 000.
                                     // So, checks M[060], M[057] ... M[010].
            const regToTest = (this.regs.sKeep2 + SBIT4) & PY_WORD_MASK; // SBIT4 is 010 octal
            const currentValue = this.memory.read(regToTest); // Simulate reading the counter/register
            const complementedValue = (~currentValue) & PY_WORD_MASK; // Simulate CS 0000
            // A real check would involve writing patterns or verifying known states.
            // Here, we just log it.
            console.debug(`Counter check: Register 0${regToTest.toString(8)}, Value 0${currentValue.toString(8)}, Complemented 0${complementedValue.toString(8)}`);
            
            if (this.regs.sKeep2 === 0) break; // Stop if SKEEP2 becomes 0. Original seems to stop earlier.
                                            // AGC CNTRLOOP: CCS SKEEP2; TC CNTRLOOP. Stops when SKEEP2 becomes 0 if it decrements.
                                            // The python code decrements counter from CNTRCON down to 0.
            this.regs.sKeep2 = (this.regs.sKeep2 - 1) & PY_WORD_MASK;
            if (this.regs.sKeep2 === (PY_WORD_MASK) ) break; // broke from negative
        }
        
        console.info("Counter check finished (simulated).");
        this.cycleAndShiftCheck(); // Proceed to CYCLSHFT
    }

    // CYCLSHFT: Check Cycle and Shift Registers
    public cycleAndShiftCheck(): void {
        if (this.alarmActive && this.regs.smode > 0) return;
        console.info("Starting cycle and shift register check (CYCLSHFT)");

        // Pattern 1: CONC_S1 (25252 octal)
        this.regs.a = CONC_S1;
        // TS CYR, TS CYL, TS SR, TS EDOP
        // These are not direct memory writes but to special CPU registers.
        // We simulate by setting properties on our regs object.
        // The values loaded are affected by the register type (e.g. CYL shifts)
        // Python directly sets them: self.regs.cyr = pattern etc. Let's assume direct set for simulation.
        this.regs.cyr = CONC_S1; // C(CYR) = 12525 (octal if it's a 15-bit right shift of 25252) - simplified
        this.regs.cyl = CONC_S1; // C(CYL) = 52524 (octal if it's a 15-bit left shift of 25252) - simplified
        this.regs.sr  = CONC_S1; // C(SR)  = 12525 (octal like CYR) - simplified
        this.regs.edop= CONC_S1; // C(EDOP)= 00125 (octal, takes low 7 bits of 25252 which is 125) - simplified
        
        // To match python behavior which directly assigns pattern:
        this.regs.cyr = CONC_S1; this.regs.cyl = CONC_S1; this.regs.sr = CONC_S1; this.regs.edop = CONC_S1;


        // AD CYR, AD CYL, AD SR, AD EDOP, AD CONC_S2 (52400 octal) -> check for -1
        let sum = (this.regs.cyr + this.regs.cyl + this.regs.sr + this.regs.edop + CONC_S2) & PY_WORD_MASK;
        if (!this.checkIfNegativeOne(sum)) {
             if (this.alarmActive && this.regs.smode > 0) return;
        }

        // AD CYR, AD CYL, AD SR, AD EDOP, AD S+1 -> check for -1
        // This is adding to previous sum or starting fresh? AGC implies it's a new sum.
        // Python: check_val = self.regs.cyr + self.regs.cyl + self.regs.sr + self.regs.edop + 1
        sum = (this.regs.cyr + this.regs.cyl + this.regs.sr + this.regs.edop + SBIT1) & PY_WORD_MASK;
        if (!this.checkIfNegativeOne(sum)) {
            if (this.alarmActive && this.regs.smode > 0) return;
        }
        
        console.info("Cycle and shift register check finished.");
        // Original: INCR SCOUNT +1; TC SMODECHK. We are in a sub-check of a larger option.
        // The main option (like FULL_CHECK) would proceed to ROPECHK if this was part of it.
        // If ERASABLE_MEMORY was chosen, this wouldn't run.
        // This is the end of the "ERASCHK path" components.
        // If this was called standalone, it would end. If part of FULL_CHECK, ROPECHK is next.
        // Since FULL_CHECK calls erasableMemoryCheck then ropeMemoryCheck, this flow is handled there.
    }

    // ROPECHK: Rope Memory Check (Fixed Memory Checksum)
    public ropeMemoryCheck(): void {
        if (this.alarmActive && this.regs.smode > 0) return;
        console.info("Starting rope memory check (ROPECHK / Fixed Memory Checksum)");

        this.regs.sKeep6 = S_ZERO_NEG; // -0 for ROPECHK. (+1 for SHOWSUM)
        this.stshosumCommonSetup(); // Common setup for ROPECHK and SHOWSUM
    }
    
    // SHOWSUM: Display bank checksums
    public showBankSum(): DskyRegisters {
        console.info("Starting SHOWSUM display mode.");
        this.regs.sKeep6 = 1; // +1 for SHOWSUM
        this.stshosumCommonSetup();
        // The loop in stshosumCommonSetup will call displaySum when sKeep6 is +1.
        // This function is more of an entry point. The result is on dskyReg.
        // A real SHOWSUM needs V33E to proceed. This simulation does all banks.
        return {...this.dskyReg};
    }

    private stshosumCommonSetup(): void {
        this.regs.sKeep4 = 0;           // Bank number (starts at 0)
        this.regs.sKeep7 = 1;           // Controls COMMFX (1) or FXFX (0 or <0). Start with Common Fixed.
        this.setupAndRunBankChecksum();
    }
    
    private setupAndRunBankChecksum(): void {
        // COMMFX part (or start of FXFX)
        if (this.regs.sKeep7 > 0) { // Common Fixed Banks
            this.regs.sKeep1 = 0; // SKEEP1 holds sum
            this.regs.sKeep3 = 0; // SKEEP3 holds present address (00000 to 01777)
        } else { // Fixed-Fixed Banks
            // FXFX part
            if (this.regs.sKeep7 === 0) { // First entry to FXFX for a bank pair
                 // Python logic: if self.regs.skeep7 == 0: self.regs.skeep3 = SBIT12 (bank 02 base)
                 // else: self.regs.skeep3 = SBNK03 (bank 03 base)
                 // This needs refinement to pick 04000 or 06000 based on superbank context.
                 // For now, assume fixed-fixed starts at 04000 (SBIT12)
                 this.regs.sKeep3 = SBIT12;
            } else { // if skeep7 < 0, it implies it was set by ~A
                 this.regs.sKeep3 = SBNK03; // Start at 06000 for other fixed-fixed parts.
            }
            this.regs.sKeep1 = 0; // Reset sum for this bank
        }
        this.regs.sKeep5 = 1; // Counts 2 successive TC SELF words (related to bugger word)

        this.processBankAddresses();
    }
    
    // Combined COMADRS and FXADRS loop logic
    private processBankAddresses(): void {
        while(true) {
            if (this.alarmActive && this.regs.smode > 0 && this.regs.sKeep6 === S_ZERO_NEG) return;

            let currentAddressToRead: number;
            if (this.regs.sKeep7 > 0) { // COMMFX
                // Address = (SKEEP4 (bank bits) shifted appropriately) + SKEEP3 (relative address)
                // SKEEP4 holds bank number. For common fixed (banks 0,1), no complex shifting.
                // Example: Bank 0, SKEEP3 = 0 to 1777. Bank 1, SKEEP3 = 0 to 1777.
                // Super Data Call in AGC: L has bank, AD SKEEP3 for address.
                // Simplified: assume SKEEP4 correctly identifies bank index for memory.fixed.
                // The memory.read will need to decode this.
                // For now, let's assume SKEEP3 is the absolute address for common fixed for simplicity
                // or SKEEP4 contains full bank prefix for memory.read.
                // Python calculates addr = (self.regs.skeep4 & 0o37000) + self.regs.skeep3
                // Let's make SKEEP4 the bank index (0-37 octal) and SKEEP3 the relative addr (0-1777 octal)
                const bankIndex = this.regs.sKeep4; // This needs to be true bank index
                currentAddressToRead = (bankIndex << 10) | this.regs.sKeep3; // Calculate true fixed mem address
            } else { // FXFX
                currentAddressToRead = this.regs.sKeep3; // SKEEP3 is already absolute 04000-07777
            }
            
            const valueRead = this.memory.read(currentAddressToRead);
            this.addToSum(valueRead); // ADSUM logic

            // ADRSCHK logic (check if current address is last for this bank)
            // Relative address is SKEEP3 & 01777
            const relativeAddr = this.regs.sKeep3 & 0o1777;

            // Checksum finished if last address (01777 octal for 1K words)
            if (relativeAddr === 0o1777) {
                this.handleRopeCheckOption(); // SOPTION logic
                return; // Finished this bank
            }

            // TC SELF word check (Simplified - Python's complex SKEEP5 logic)
            // Original checks for TC SELF (opcode 06xxxx) to find "bugger words".
            // This simulation won't deeply inspect opcodes.
            // Python's self.regs.skeep5 counts down. If it's zero, call soption.
            // This seems to be for handling bugger words correctly in checksum.
            this.regs.sKeep5 = (this.regs.sKeep5 - 1) & PY_WORD_MASK;
            if (this.regs.sKeep5 === 0 || this.regs.sKeep5 === PY_WORD_MASK) { // Counted down or wrapped around
                 // This is a simplification of the "bugger word" and TC SELF logic
                 // If we encounter this, assume it's time to check/finalize bank per python.
                this.handleRopeCheckOption();
                return; 
            }
            
            // ADRS+1: Increment SKEEP3 (current address)
            this.regs.sKeep3 = (this.regs.sKeep3 + 1) & PY_WORD_MASK;
            // Loop continues
        }
    }
    
    // ADSUM: Add value to checksum (SKEEP1)
    private addToSum(value: number): void {
        this.regs.sKeep2 = value; // Store current value read
        this.regs.sKeep1 = (this.regs.sKeep1 + value) & PY_WORD_MASK;
        // Original: CAF S+ZERO; AD SKEEP1; TS SKEEP1. This is redundant if SKEEP1 already holds sum.
        // The final CS SKEEP2; AD SKEEP3; TC Q part of ADSUM is not directly modelled here,
        // as TC Q means a jump based on Q register, not a return.
    }
    
    // SOPTION: Handle checksum verification or display
    private handleRopeCheckOption(): void {
        let bankNumberToDisplay = 0;
        // Derive bank number for display/check from SKEEP4 and L (Superbank logic)
        // Original: CA SKEEP4; MASK HI5; TC LEFT5; TS L (Bank before super)
        //           CA SKEEP4; MASK S8BITS; BZF SOPT (Superbank bits)
        //           TS SR; CA L; MASK SEVEN; AD SR; TS L (Bank with super)
        // Simplified from Python:
        const rawBankBitsFromSkeep4 = (this.regs.sKeep4 & 0o37000) >> 10; // HI5 type logic
        const superBankBitsFromSkeep4 = this.regs.sKeep4 & 0o00377; // S8BITS type logic
        
        this.regs.l = rawBankBitsFromSkeep4; // Store base bank in L
        if (superBankBitsFromSkeep4 !== 0) {
            this.regs.sr = superBankBitsFromSkeep4; // Store super bank in SR
            this.regs.l = (this.regs.l & 0o7) + this.regs.sr; // Bank num with super bank
        }
        bankNumberToDisplay = this.regs.l;


        if (this.regs.sKeep6 !== S_ZERO_NEG) { // SHOWSUM mode (+1 in sKeep6)
            this.displaySum(bankNumberToDisplay);
        } else { // ROPECHK mode (-0 in sKeep6)
            // Force sum to absolute value (BNKCHK part)
            let sumToCheck = this.regs.sKeep1;
            if ((sumToCheck & 0o40000) !== 0) { // Check sign bit for 15-bit 1's complement
                sumToCheck = ((~sumToCheck) & PY_WORD_MASK) + SBIT1; // approx 2's complement for abs
                // Or if 1's comp: sumToCheck = (~sumToCheck) & AGC_WORD_MASK
            }
             // Python: if self.regs.skeep1 < 0: self.regs.skeep1 = -self.regs.skeep1
             // For PY_WORD_MASK (unsigned style), "negative" means top bit set.
             // A true absolute for 1's complement is more complex.
             // Simplification: if sum looks "negative", make it "positive" for check.
             // The AGC check is: CS L (-BankNum) + AD SKEEP1 (Sum) + AD S-1 (-1) == -1 ?
             // Which means Sum - BankNum - 1 == -1  => Sum == BankNum
            
            const checkVal = ( ( (~bankNumberToDisplay) & PY_WORD_MASK) + sumToCheck + ((~SBIT1)&PY_WORD_MASK) ) & PY_WORD_MASK;

            if (!this.checkIfNegativeOne(checkVal)) {
                console.error(`ROPECHK failed for bank ${bankNumberToDisplay.toString(8)}. Expected sum ~${bankNumberToDisplay.toString(8)}, Got ~${this.regs.sKeep1.toString(8)}`);
                // Error is handled by checkIfNegativeOne
                if (this.alarmActive && this.regs.smode > 0) return;
            } else {
                 console.info(`ROPECHK success for bank ${bankNumberToDisplay.toString(8)}.`);
            }
        }
        this.moveToNextBank();
    }
    
    // SDISPLAY (part of SOPTION for SHOWSUM)
    private displaySum(currentBankNum: number): void {
        this.dskyReg.R1 = this.regs.sKeep1; // Bank Sum
        this.dskyReg.R2 = currentBankNum;   // Bank Number (from L)
        this.dskyReg.R3 = this.regs.sKeep3; // Bugger Word (last address or related value)
        console.log(`SHOWSUM: Bank 0${currentBankNum.toString(8)} Sum=0${this.dskyReg.R1.toString(8)} BuggerWord=0${this.dskyReg.R3.toString(8)}`);
    }

    // NXTBNK: Move to the next bank for checking
    private moveToNextBank(): void {
        // Check if LSTBNKCH is reached
        // Original: CS SKEEP4; AD LSTBNKCH; BZF ENDSUMS
        // LSTBNKCH is a constant (e.g. 040 for last bank number)
        const lastBankToCheck = 0o37; // Luminary has banks 0-37 octal (31 decimal)
                                   // Python used 0o40. Let's use 0o37 as a more common AGC max.
        
        if (this.regs.sKeep4 === lastBankToCheck) {
            this.handleEndOfSums();
            return;
        }

        // Increment bank logic (complex in AGC involving SBIT11, SBIT15, SIXTY, SUPRCON)
        // Python: self.regs.skeep4 += SBIT11 then _chksupr
        // This simulates advancing SKEEP4 to the next bank index.
        // A simple increment for now, actual AGC logic is for physical bank mapping.
        this.regs.sKeep4 = (this.regs.sKeep4 + 1) & PY_WORD_MASK; 

        // CHKSUPR logic and GONXTBNK
        // This part is very hardware specific. Python simplifies.
        // We choose COMMFX or FXFX based on the new SKEEP4.
        // Typically, low banks are COMMFX (erasable mapped), higher are FXFX.
        // Let's assume banks 0,1 are common, others are fixed-fixed for this simulation.
        if (this.regs.sKeep4 <= 1) { // Example threshold for common fixed
            this.regs.sKeep7 = 1; // COMMFX
        } else {
            this.regs.sKeep7 = 0; // FXFX (will set to SBIT12 or SBNK03 in setup)
        }
        this.setupAndRunBankChecksum(); // Re-enter checksum for new bank
    }
    
    // ENDSUMS: Handle completion of memory checksums
    private handleEndOfSums(): void {
        console.info("All bank checksums (ROPECHK/SHOWSUM) complete.");
        // If in self-check, this might be the end of FIXED_MEMORY portion.
        // TC SMODECHK in some paths of original.
    }

    // Public method to run self-check with a specific mode
    public runSelfCheck(mode: SelfCheckMode): { errorCount: number; alarmActive: boolean; failReg: number[]; dsky: DskyRegisters } {
        this.regs.reset(); // Reset registers for a fresh run
        this.alarmActive = false;
        this.failReg = [0,0,0];
        
        this.regs.smode = mode;
        if (mode === SelfCheckMode.CONTINUE_ON_ERROR) {
            // Convert special -0 to a numeric value that runSelectedOption can map
            // Python does this by setting effectiveOption.
            // For smode itself, it should remain a distinct value if possible.
            // For now, the logic inside runSelectedOption handles mapping -0.
        }

        this.selfCheckEntry(); // Start the check process

        return {
            errorCount: this.regs.erCount,
            alarmActive: this.alarmActive,
            failReg: [...this.failReg],
            dsky: {...this.dskyReg}
        };
    }
}


// Example Usage (similar to Python's if __name__ == "__main__":)
function main(): void {
    console.log("=== AGC Block Two Self-Check (TypeScript) ===");

    const agcCheck = new AGCSelfCheck();

    // Example: Run Erasable Memory Check
    console.log("\nRunning ERASABLE_MEMORY check...");
    let results = agcCheck.runSelfCheck(SelfCheckMode.ERASABLE_MEMORY);
    if (results.errorCount > 0) {
        console.error(`Erasable Memory Check failed with ${results.errorCount} errors. Alarm: ${results.alarmActive}`);
    } else {
        console.log("Erasable Memory Check completed successfully.");
    }

    // Example: Run Fixed Memory (Rope) Check
    // To make this meaningful, memory.fixed should be populated with some data.
    // For now, it will check sums of zero-filled banks.
    console.log("\nRunning FIXED_MEMORY (ROPECHK) check...");
    // Re-initialize for a new check type if desired, or continue state
    // const agcFreshCheck = new AGCSelfCheck(); // For a fully fresh state
    results = agcCheck.runSelfCheck(SelfCheckMode.FIXED_MEMORY); 
    if (results.errorCount > 0) {
        console.error(`Fixed Memory Check failed with ${results.errorCount} errors. Alarm: ${results.alarmActive}`);
    } else {
        console.log("Fixed Memory Check completed successfully (ran on empty banks).");
    }
    
    // Example: Show Bank Sums (will also run on empty banks here)
    console.log("\nRunning SHOW_BANKSUM...");
    const agcShowSum = new AGCSelfCheck(); // Use a fresh instance for SHOWSUM
    agcShowSum.showBankSum(); // Output will be logged by displaySum method
    console.log("SHOW_BANKSUM finished (ran on empty banks). DSKY R1-R3 reflect last bank (or initial state).");
    console.log("DSKY from showBankSum instance:", agcShowSum.dskyReg);


    // Example: Full check (includes both erasable and fixed)
    console.log("\nRunning FULL_CHECK_10 (Erasable + Fixed)...");
    const agcFullCheck = new AGCSelfCheck();
    results = agcFullCheck.runSelfCheck(SelfCheckMode.FULL_CHECK_10);
     if (results.errorCount > 0) {
        console.error(`Full Check failed with ${results.errorCount} errors. Alarm: ${results.alarmActive}`);
    } else {
        console.log("Full Check completed successfully (fixed memory ran on empty banks).");
    }
}

// To run this example if this file is executed directly (e.g. with ts-node or after compiling to JS)
// In a module system, you would typically import and use the AGCSelfCheck class.
// For direct execution simulation:
// if (typeof require !== 'undefined' && require.main === module) { // Removed to avoid Node-specific linting issues
//     main();
// }

// You can call main() directly if running in an environment where it's appropriate
// For example, in a script executed by ts-node, or after compilation to JS and run with Node.
// main(); // Uncomment to run example automatically when file is executed.
