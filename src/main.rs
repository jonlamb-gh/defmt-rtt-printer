use clap::Parser;
use defmt_decoder::{DecodeError, Frame, Location, Locations, Table};
use probe_rs::{
    config::MemoryRegion,
    probe::{list::Lister, DebugProbeSelector, WireProtocol},
    rtt::{ChannelMode, Rtt, ScanRegion},
    Core, CoreStatus, HaltReason, Permissions, RegisterValue, VectorCatchCondition,
};
use std::{
    env, fs, io,
    path::{Path, PathBuf},
    sync::atomic::{AtomicBool, Ordering::SeqCst},
    sync::Arc,
    time::{Duration, Instant},
};
use tracing::{debug, error, warn};

/// Connect to a target and print defmt frames from an RTT buffer
#[derive(Parser, Debug, Clone)]
#[clap(version)]
#[command(name = "defmt-rtt-printer")]
struct Opts {
    /// Specify a target attach timeout.
    /// When provided, the plugin will continually attempt to attach and search
    /// for a valid RTT control block anywhere in the target RAM.
    ///
    /// Accepts durations like "10ms" or "1minute 2seconds 22ms".
    #[clap(long, name = "attach-timeout")]
    pub attach_timeout: Option<humantime::Duration>,

    /// Use the provided RTT control block address instead of scanning the target memory for it.
    #[clap(long, name = "control-block-address")]
    pub control_block_address: Option<u32>,

    /// The RTT up (target to host) channel number to poll on (defaults to 0).
    #[clap(long, name = "up-channel", default_value = "0")]
    pub up_channel: usize,

    /// Select a specific probe instead of opening the first available one.
    ///
    /// Use '--probe VID:PID' or '--probe VID:PID:Serial' if you have more than one probe with the same VID:PID.
    #[structopt(long = "probe", name = "probe")]
    pub probe_selector: Option<DebugProbeSelector>,

    /// The target chip to attach to (e.g. STM32F407VE).
    #[clap(long, name = "chip")]
    pub chip: String,

    /// Protocol used to connect to chip.
    /// Possible options: [swd, jtag].
    ///
    /// The default value is swd.                                                                                                       
    #[structopt(long, name = "protocol", default_value = "Swd")]
    pub protocol: WireProtocol,

    /// The protocol speed in kHz.                                                                                                      
    ///                                                                                                
    /// The default value is 4000.                                                                     
    #[clap(long, name = "speed", default_value = "4000")]
    pub speed: u32,

    /// The selected core to target.
    ///
    /// The default value is 0.
    #[clap(long, name = "core", default_value = "0")]
    pub core: usize,

    /// Reset the target on startup.
    #[clap(long, name = "reset")]
    pub reset: bool,

    /// Attach to the chip under hard-reset.
    #[clap(long, name = "attach-under-reset")]
    pub attach_under_reset: bool,

    /// Chip description YAML file path.
    /// Provides custom target descriptions based on CMSIS Pack files.
    #[clap(long, name = "chip-description-path")]
    pub chip_description_path: Option<PathBuf>,

    /// Show verbose information (location info, etc)
    #[arg(short, long)]
    pub verbose: bool,

    /// Set a breakpoint on the address of the given symbol when
    /// enabling RTT BlockIfFull channel mode.
    ///
    /// Can be an absolute address or symbol name.
    #[arg(long)]
    pub breakpoint: Option<String>,

    /// Assume thumb mode when resolving symbols from the ELF file
    /// for breakpoints.
    #[arg(long, requires = "breakpoint")]
    pub thumb: bool,

    /// The ELF file containing the defmt table and location information.
    #[clap(name = "elf-file", verbatim_doc_comment)]
    pub elf_file: PathBuf,
}

fn main() {
    let opts = Opts::parse();

    tracing_subscriber::fmt::init();

    let intr = Interruptor::new();
    let intr_clone = intr.clone();
    ctrlc::set_handler(move || {
        if intr_clone.is_set() {
            let exit_code = if cfg!(target_family = "unix") {
                // 128 (fatal error signal "n") + 2 (control-c is fatal error signal 2)
                130
            } else {
                // Windows code 3221225786
                // -1073741510 == C000013A
                -1073741510
            };
            std::process::exit(exit_code);
        }

        debug!("Shutdown signal received");
        intr_clone.set();
    })
    .expect("ctrlc handler");

    let current_dir = env::current_dir().unwrap();

    debug!(elf_file = %opts.elf_file.display(), "Reading ELF file");
    let elf_contents = fs::read(&opts.elf_file).expect("Read ELF file");

    debug!("Reading defmt table");
    let table = Table::parse(&elf_contents)
        .unwrap()
        .expect("Parse defmt table");

    let mut buffer = vec![0_u8; 1024];
    let mut decoder = table.new_stream_decoder();

    let location_info = {
        // This is essentially what probe-rs reports to the user
        let locs = table.get_locations(&elf_contents).unwrap();
        if !table.is_empty() && locs.is_empty() {
            warn!("Insufficient DWARF info; compile your program with `debug = 2` to enable location info.");
            None
        } else if table.indices().all(|idx| locs.contains_key(&(idx as u64))) {
            Some(locs)
        } else {
            warn!("Location info is incomplete; it will be omitted when constructing event attributes.");
            None
        }
    };

    if let Some(chip_desc) = &opts.chip_description_path {
        debug!(path = %chip_desc.display(), "Adding custom chip description");
        let f = fs::File::open(chip_desc).expect("chip-desc file open");
        probe_rs::config::add_target_from_yaml(f).unwrap();
    }

    let lister = Lister::new();
    let mut probe = if let Some(probe_selector) = &opts.probe_selector {
        debug!(probe_selector = %probe_selector, "Opening selected probe");
        lister.open(probe_selector.clone()).expect("listen open")
    } else {
        let probes = lister.list_all();
        debug!(probes = probes.len(), "Opening first available probe");
        if probes.is_empty() {
            panic!("No probes available");
        }
        probes[0].open(&lister).expect("probe open")
    };

    debug!(protocol = %opts.protocol, speed = opts.speed, "Configuring probe");
    probe.select_protocol(opts.protocol).unwrap();
    probe.set_speed(opts.speed).unwrap();

    debug!(chip = opts.chip, core = opts.core, "Attaching to chip");

    let mut session = if opts.attach_under_reset {
        probe
            .attach_under_reset(opts.chip, Permissions::default())
            .expect("probe attach session")
    } else {
        probe
            .attach(opts.chip, Permissions::default())
            .expect("probe attach session")
    };

    let rtt_scan_regions = session.target().rtt_scan_regions.clone();
    let mut rtt_scan_region = if rtt_scan_regions.is_empty() {
        ScanRegion::Ram
    } else {
        ScanRegion::Ranges(rtt_scan_regions)
    };
    if let Some(user_provided_addr) = opts.control_block_address {
        debug!(
            rtt_addr = user_provided_addr,
            "Using explicit RTT control block address"
        );
        rtt_scan_region = ScanRegion::Exact(user_provided_addr);
    } else {
        let mut file = fs::File::open(&opts.elf_file).expect("open elf file");
        if let Some(rtt_addr) = get_rtt_symbol(&mut file) {
            debug!(rtt_addr = rtt_addr, "Found RTT symbol");
            rtt_scan_region = ScanRegion::Exact(rtt_addr as _);
        }
    }

    let memory_map = session.target().memory_map.clone();

    let mut core = session.core(opts.core).unwrap();

    let core_status = core.status().unwrap();
    debug!(status = ?core_status, "core status");

    if opts.reset {
        debug!("Reset and halt core");
        core.reset_and_halt(Duration::from_millis(100)).unwrap();
    }

    // Disable any previous vector catching (i.e. user just ran probe-rs run or a debugger)
    core.disable_vector_catch(VectorCatchCondition::All)
        .unwrap();
    core.clear_all_hw_breakpoints().unwrap();

    if let Some(bp_sym_or_addr) = opts.breakpoint.as_ref() {
        let num_bp = core.available_breakpoint_units().unwrap();
        assert!(num_bp > 0, "No breakpoints available");

        let bp_addr = if let Some(bp_addr) = bp_sym_or_addr
            .parse::<u64>()
            .ok()
            .or(u64::from_str_radix(bp_sym_or_addr.trim_start_matches("0x"), 16).ok())
        {
            bp_addr
        } else {
            let mut file = fs::File::open(&opts.elf_file).expect("open elf file");
            let bp_addr = get_symbol(&mut file, bp_sym_or_addr).unwrap();
            if opts.thumb {
                bp_addr & !1
            } else {
                bp_addr
            }
        };

        debug!(
            available_breakpoints = num_bp,
            symbol_or_addr = bp_sym_or_addr,
            addr = format_args!("0x{:X}", bp_addr),
            "Setting breakpoint"
        );
        core.set_hw_breakpoint(bp_addr).unwrap();
    }

    let mut rtt = match opts.attach_timeout {
        Some(to) if !to.is_zero() => {
            attach_retry_loop(&mut core, &memory_map, &rtt_scan_region, to).unwrap()
        }
        _ => {
            debug!("Attaching to RTT");
            Rtt::attach_region(&mut core, &memory_map, &rtt_scan_region).unwrap()
        }
    };

    let up_channel = rtt
        .up_channels()
        .take(opts.up_channel)
        .expect("take up channel");
    let up_channel_mode = up_channel.mode(&mut core).unwrap();
    let up_channel_name = up_channel.name().unwrap_or("NA");
    debug!(channel = up_channel.number(), name = up_channel_name, mode = ?up_channel_mode, buffer_size = up_channel.buffer_size(), "Opened up channel");

    if opts.reset || opts.attach_under_reset {
        let sp_reg = core.stack_pointer();
        let sp: RegisterValue = core.read_core_reg(sp_reg.id()).unwrap();
        let pc_reg = core.program_counter();
        let pc: RegisterValue = core.read_core_reg(pc_reg.id()).unwrap();
        debug!(pc = %pc, sp = %sp, "Run core");
        core.run().unwrap();
    }

    if opts.breakpoint.is_some() {
        debug!("Waiting for breakpoint");
        'bp_loop: loop {
            if intr.is_set() {
                break;
            }

            match core.status().unwrap() {
                CoreStatus::Running => (),
                CoreStatus::Halted(halt_reason) => match halt_reason {
                    HaltReason::Breakpoint(_) => break 'bp_loop,
                    _ => {
                        warn!(reason = ?halt_reason, "Unexpected halt reason");
                        break 'bp_loop;
                    }
                },
                state => panic!("Core is in an unexpected state {state:?}"),
            }

            std::thread::sleep(Duration::from_millis(100));
        }

        let mode = ChannelMode::BlockIfFull;
        debug!(mode = ?mode, "Set channel mode");
        up_channel.set_mode(&mut core, mode).unwrap();

        debug!("Run core post breakpoint");
        core.run().unwrap();
    }

    loop {
        if intr.is_set() {
            break;
        }
        let rtt_bytes_read = up_channel
            .read(&mut core, &mut buffer)
            .expect("RTT channel read");

        decoder.received(&buffer[..rtt_bytes_read]);
        'read_loop: loop {
            let frame = match decoder.decode() {
                Ok(f) => f,
                Err(e) => match e {
                    DecodeError::UnexpectedEof => {
                        // Need more data
                        break 'read_loop;
                    }
                    DecodeError::Malformed => {
                        warn!("Malformed defmt frame");
                        continue;
                    }
                },
            };

            println!("{}", frame.display(true));
            if opts.verbose {
                if let Some(loc) = get_location_info(&location_info, &frame, &current_dir) {
                    println!("└─ {} @ {}:{}", loc.module, loc.file.display(), loc.line);
                }
            }
        }

        // NOTE: this is what probe-rs does
        //
        // Poll RTT with a frequency of 10 Hz if we do not receive any new data.
        // Once we receive new data, we bump the frequency to 1kHz.
        //
        // If the polling frequency is too high, the USB connection to the probe
        // can become unstable. Hence we only pull as little as necessary.
        if rtt_bytes_read != 0 {
            std::thread::sleep(Duration::from_millis(1));
        } else {
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    debug!("Shutting down");

    up_channel
        .set_mode(&mut core, ChannelMode::NoBlockTrim)
        .unwrap();
}

fn get_location_info(
    locs: &Option<Locations>,
    frame: &Frame,
    current_dir: &Path,
) -> Option<Location> {
    let loc = locs.as_ref().map(|locs| locs.get(&frame.index()));

    if let Some(Some(loc)) = loc {
        // try to get the relative path, else the full one
        let path = loc.file.strip_prefix(current_dir).unwrap_or(&loc.file);

        Some(Location {
            file: path.to_owned(),
            line: loc.line,
            module: loc.module.clone(),
        })
    } else {
        None
    }
}

fn get_rtt_symbol<T: io::Read + io::Seek>(file: &mut T) -> Option<u64> {
    get_symbol(file, "_SEGGER_RTT")
}

fn get_symbol<T: io::Read + io::Seek>(file: &mut T, symbol: &str) -> Option<u64> {
    let mut buffer = Vec::new();
    if file.read_to_end(&mut buffer).is_ok() {
        if let Ok(binary) = goblin::elf::Elf::parse(buffer.as_slice()) {
            for sym in &binary.syms {
                if let Some(name) = binary.strtab.get_at(sym.st_name) {
                    if name == symbol {
                        return Some(sym.st_value);
                    }
                }
            }
        }
    }
    None
}

fn attach_retry_loop(
    core: &mut Core,
    memory_map: &[MemoryRegion],
    scan_region: &ScanRegion,
    timeout: humantime::Duration,
) -> Option<Rtt> {
    debug!(timeout = %timeout, "Attaching to RTT");
    let timeout: Duration = timeout.into();
    let start = Instant::now();
    while Instant::now().duration_since(start) <= timeout {
        match Rtt::attach_region(core, memory_map, scan_region) {
            Ok(rtt) => return Some(rtt),
            Err(e) => {
                if matches!(e, probe_rs::rtt::Error::ControlBlockNotFound) {
                    std::thread::sleep(Duration::from_millis(50));
                    continue;
                }

                error!("Failed to attach to RTT");
                return None;
            }
        }
    }

    // Timeout reached
    warn!("Timed out attaching to RTT");
    Some(Rtt::attach(core, memory_map).expect("RTT attach"))
}

#[derive(Clone, Debug)]
#[repr(transparent)]
pub struct Interruptor(Arc<AtomicBool>);

impl Interruptor {
    pub fn new() -> Self {
        Interruptor(Arc::new(AtomicBool::new(false)))
    }

    pub fn set(&self) {
        self.0.store(true, SeqCst);
    }

    pub fn is_set(&self) -> bool {
        self.0.load(SeqCst)
    }
}

impl Default for Interruptor {
    fn default() -> Self {
        Self::new()
    }
}
