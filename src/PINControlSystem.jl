module PINControlSystem
  using DataFrames
  using Unitful
  import DimensionfulAngles: Angle, radᵃ, °ᵃ
  using CSV
  using PI5_LG_IO

  # Exported functions
  export lg_gpiochip_open, lg_spi_open, lg_gpio_write, lg_spi_write, lg_spi_read, lg_error_text, lg_gpiochip_close, lg_spi_close, lg_gpio_free, lg_gpio_claim_output, lg_gpio_claim_input, lg_gpio_claim_input_pullup, lg_spi_xfer, lg_spi_read, lg_spi_write
  export PINController, PINControllerSystem, put_board!, lg_set_gpio_output, lg_close, set_config, matching_antenna_connectors!,
          get_board_by_antconnector, get_board_by_PINn, select_board, deselect_board, get_spis, send_spi, read_spi, 
          get_bids, get_cconfig, get_cnumbers, get_board, change_pid_states!, lg_close, getbycid, getbybid, getbybport, 
          put_pin_state_bybid, get_active_pins, put_pin_all_state!, send_pin_states, put_intensity_bybid!, put_intensity!, 
          get_pin_state, send_intensity_states, example, Angle, radᵃ, °ᵃ,
          set_display_test_mode, set_current_mode, set_global_current, send_spi_selective_chips

  # Register Address Map for MAX6957
  const REG_NO_OP = 0x00
  const REG_GLOBAL_CURRENT = 0x02 # 0x00 ~ 0x0F current level, 0x0F: 16/16 Max
  const REG_CONFIG = 0x04 # D0:OPERATION[Shutdown(LOW),NORMAL(HIGH)], D6:Current[Global(LOW),INDIVISUAL(HIGH)], D7:Transition Detection[Disabled(LOW), Enabled(HIGH)]
  const REG_PORT_CONFIG = 0x06
  const REG_TRANSITION_DETECT_MASK = 0x06
  const REG_DISPLAY_TEST = 0x07 # D0: Display Test Mode[Normal(LOW), Test(HIGH)]

  # Configuration bits for MAX6957
  const CONFIG_SHUTDOWN = 0x00
  const CONFIG_NORMAL = 0x01
  const CONFIG_GLOBAL_CURRENT = 0x00
  const CONFIG_INDIVIDUAL_CURRENT = 0x40
  const CONFIG_DETECT_DISABLED = 0x00
  const CONFIG_DETECT_ENABLED = 0x80

  # Port Configuration Registers for MAX6957
  const REG_PORT_CONFIG_1 = 0x09   # P7, P6, P5, P4 
  const REG_PORT_CONFIG_2 = 0x0A   # P11, P10, P9, P8
  const REG_PORT_CONFIG_3 = 0x0B   # P15, P14, P13, P12
  const REG_PORT_CONFIG_4 = 0x0C   # P19, P18, P17, P16
  const REG_PORT_CONFIG_5 = 0x0D   # P23, P22, P21, P20
  const REG_PORT_CONFIG_6 = 0x0E   # P27, P26, P25, P24
  const REG_PORT_CONFIG_7 = 0x0F   # P31, P30, P29, P28

  # Port configuration bits for MAX6957
  const PORT_CONFIG_LED_DRIVER = 0x00
  const PORT_CONFIG_GPIO_OUTPUT = 0x01
  const PORT_CONFIG_GPIO_INPUT = 0x02
  const PORT_CONFIG_GPIO_INPUT_PULLUP = 0x03

  # Current Registers for MAX6957
  const REG_CURRENT_054 = 0x12  # 5: 0x0X ~ 0xFX current level, 4: 0xX0~0xXF current level
  const REG_CURRENT_076 = 0x13  # 7: 0x0X ~ 0xFX current level, 6: 0xX0~0xXF current level
  const REG_CURRENT_098 = 0x14  # 9: 0x0X ~ 0xFX current level, 8: 0xX0~0xXF current level
  const REG_CURRENT_0BA = 0x15  # 11: 0x0X ~ 0xFX current level, 10: 0xX0~0xXF current level
  const REG_CURRENT_0DC = 0x16  # 13: 0x0X ~ 0xFX current level, 12: 0xX0~0xXF current level
  const REG_CURRENT_0FE = 0x17  # 15: 0x0X ~ 0xFX current level, 14: 0xX0~0xXF current level
  const REG_CURRENT_110 = 0x18  # 16: 0x0X ~ 0xFX current level, 17: 0xX0~0xXF current level
  const REG_CURRENT_132 = 0x19  # 18: 0x0X ~ 0xFX current level, 19: 0xX0~0xXF current level
  const REG_CURRENT_154 = 0x1A  # 20: 0x0X ~ 0xFX current level, 21: 0xX0~0xXF current level
  const REG_CURRENT_176 = 0x1B  # 22: 0x0X ~ 0xFX current level, 23: 0xX0~0xXF current level
  const REG_CURRENT_198 = 0x1C  # 24: 0x0X ~ 0xFX current level, 25: 0xX0~0xXF current level
  const REG_CURRENT_1BA = 0x1D  # 26: 0x0X ~ 0xFX current level, 27: 0xX0~0xXF current level
  const REG_CURRENT_1DC = 0x1E  # 28: 0x0X ~ 0xFX current level, 29: 0xX0~0xXF current level
  const REG_CURRENT_1FE = 0x1F  # 30: 0x0X ~ 0xFX current level, 31: 0xX0~0xXF current level

  # 8 port registers for MAX6957
  const REG_4_11 = 0x44     # 8 ports, 4–11 (data bits D0–D7) 
  const REG_12_19 = 0x4C    # 8 ports, 12-19 (data bits D0–D7) 
  const REG_20_27 = 0x54    # 8 ports, 20-27 (data bits D0–D7) 
  const REG_28_31 = 0x5C    # 4 ports, 28-31 (data bits D0–D3. D4–D7 read as 0) 

  const CHIPs = 11

  """
      lg_set_gpio_output(gpio_handle, pin)

  Configures a specific GPIO pin as an output.

  # Arguments
  - `gpio_handle`: Handle to the GPIO chip.
  - `pin`: The GPIO pin number to configure.
  """
  function lg_set_gpio_output(gpio_handle, pin)
    result = lg_gpio_claim_output(gpio_handle, 0, pin, 1)
    if result < 0
      error("Failed to claim GPIO pin $pin : $(lg_error_text(result))")
    end
  end

  """
      lg_close()

  Closes all open GPIO and SPI handles and frees claimed pins.
  This function relies on global variables `gpio_handle`, `spi0_handle`, etc., which is not ideal for library code but fits the current usage pattern.
  """
  function lg_close()
    global gpio_handle, spi0_handle, spi1_handle, spi0_cs0_pin, spi0_cs1_pin, spi0_cs2_pin, spi1_cs0_pin, spi1_cs1_pin, spi1_cs2_pin
    a = lg_gpiochip_close(gpio_handle)
    if a < 0
      error("Failed to close GPIO handle: $(lg_error_tex(a))")
    end
    a = lg_spi_close(spi0_handle)
    if a < 0
      error("Failed to close SPI0 handle: $(lg_error_tex(a))")
    end
    a = lg_spi_close(spi1_handle)
    if a < 0
      error("Failed to close SPI1 handle: $(lg_error_tex(a))")
    end
    a = lg_gpio_free(gpio_handle, spi0_cs0_pin)
    if a < 0
      error("Failed to free GPIO pin $spi0_cs0_pin: $(lg_error_tex(a))")
    end
    a = lg_gpio_free(gpio_handle, spi0_cs1_pin)
    if a < 0
      error("Failed to free GPIO pin $spi0_cs1_pin: $(lg_error_tex(a))")
    end
    a = lg_gpio_free(gpio_handle, spi0_cs2_pin)
    if a < 0
      error("Failed to free GPIO pin $spi0_cs2_pin: $(lg_error_tex(a))")
    end
    a = lg_gpio_free(gpio_handle, spi1_cs0_pin)
    if a < 0
      error("Failed to free GPIO pin $spi1_cs0_pin: $(lg_error_tex(a))")
    end
    a = lg_gpio_free(gpio_handle, spi1_cs1_pin)
    if a < 0
      error("Failed to free GPIO pin $spi1_cs1_pin: $(lg_error_tex(a))")
    end
    a = lg_gpio_free(gpio_handle, spi1_cs2_pin)
    if a < 0
      error("Failed to free GPIO pin $spi1_cs2_pin: $(lg_error_tex(a))")
    end
  end

  """
      PINController

  Mutable struct representing the PIN diode control system.

  # Fields
  - `gpio_handle::Int32`: Handle to the GPIO chip.
  - `riset::Unitful.ElectricalResistance`: Resistor value for current setting (default 93.1kΩ).
  - `df::DataFrame`: Internal DataFrame storing the state and configuration of all pins.
  """
  mutable struct PINController
    gpio_handle::Int32
    riset::Unitful.ElectricalResistance
    df::DataFrame
    
    function PINController(gpioH::Integer, riset::Unitful.ElectricalResistance=93.1u"kΩ")
      # gpio_handle = lg_gpiochip_open(gpio_chip)
      if gpioH < 0
        error("Failed to open GPIO chip: $(lg_error_text(gpio_handle))")
      end
      df = DataFrame(pid=Int[],bid=Int[],spi_handle=Int32[],cs_pin=Int32[],bport=Int[],
        cid=Int[],cconfig=UInt8[], cport=Int[],cpin_enable=Bool[],
        cpin_mode=UInt8[],cpin_state=Bool[],cpin_intensity=UInt8[],cpin_current=Vector{typeof(1.0u"A")}())
      new(gpioH, riset, df)
    end  
  end

  """
      PINControllerSystem(gpioH::Integer, riset::Unitful.ElectricalResistance=93.1u"kΩ")

  Constructor wrapper for `PINController`.
  """
  function PINControllerSystem(gpioH::Integer, riset::Unitful.ElectricalResistance=93.1u"kΩ")
    return PINController(gpioH, riset)
  end

  """
      put_board!(CS::PINController, boardid::Int, spi_handle::Integer, cs_pin::Integer)

  Adds a new board to the PIN controller system.
  Each board consists of 11 daisy-chained MAX6957 chips.

  # Arguments
  - `CS`: The PIN controller instance.
  - `boardid`: Unique identifier for the board.
  - `spi_handle`: SPI handle for communication.
  - `cs_pin`: GPIO pin used for Chip Select.
  """
  function put_board!(CS::PINController, boardid::Int, spi_handle::Integer, cs_pin::Integer)
    if isempty(CS.df[:, :bid])
      bbid = 1
      ppid = 1
    else
      if !isempty(CS.df[CS.df.bid .== boardid,:]) 
        @show "Same board ID already exists. replace it"
        deleteat!(cs.df, findall(cs.df[:, :bid] .== boardid))
      end  
      bbid = boardid #unique(CS.df[:, :bid])[end] + 1
      ppid = CS.df[CS.df.cpin_enable .== true, :pid][end] + 1
    end

    chip_pin_map = Dict(4 => 27, 5 => 25, 6 => 23, 7 => 21, 8 => 0, 9 => 2, 10 => 4, 11 => 6, 12 => 1, 13 => 3, 14 => 5, 15 => 7,
      16 => 8, 17 => 9, 18 => 10, 19 => 11, 20 => 12, 21 => 13, 22 => 14, 23 => 15, 24 => 16, 25 => 17, 26 => 18,
      27 => 19, 28 => 20, 29 => 22, 30 => 24, 31 => 26)
    df = sort!(DataFrame(cport=collect(keys(chip_pin_map)), bport=collect(values(chip_pin_map))), :bport)
    config_value = CONFIG_NORMAL
    config_value |= CONFIG_INDIVIDUAL_CURRENT
    config_value |= CONFIG_DETECT_DISABLED

    
    curt = 936u"kΩ" / CS.riset  * u"mA"
    for i = 1:CHIPs
      boardmap = sort!(DataFrame(
        pid=collect(ppid:ppid+27) .+ (i - 1) * 28, # System PIN diode id
        spi_handle = Int32(spi_handle),            # SPI handle for the board 
        cs_pin=Int32(cs_pin),                     # CS pin for the board
        bid=bbid,                              # Board ID (1~6)
        bport=df.bport .+ (i - 1) * 28,  # Board port number (1-300)
        cid = i,                                # Chip ID (1-11)
        cconfig=[config_value for i = 1:28],    # Chip state (0x00: SHUTDOWN, 0x01: NORMAL, 0x40: INDIVIDUAL_CURRENT, )
        cport=df.cport,      # Chip port number (4-31)
        cpin_enable=[true for i = 1:28],        # Pin enable (0: disable, 1: enable)
        cpin_mode=[PORT_CONFIG_GPIO_OUTPUT for i = 1:28],          # Pin mode (0: LED driver, 1: GPIO output, 2: GPIO input, 3: GPIO input with pull-up)
        cpin_state=[false for i = 1:28],         # Pin state (0: low, 1: high)
        cpin_intensity=[0x00 for i = 1:28],      # Pin intensity (0~15)
        cpin_current=[curt for i = 1:28]))       # Pin current (voltage(5V/93.5kohm) / pin number )
      if i == CHIPs
        boardmap[end-7:end, :cpin_enable] .= false
      end    
      #sort!(boardmap, :bport)
      #print(i, boardmap)#boardmap = transform(cpinmap, :pid => ByRow(x -> x + 1) => :pid)
        
      #cs.df = vcat(cs.df, boardmap)
      CS.df = vcat(CS.df, boardmap)
    end
  end

  """
      set_config(cs::PINController)

  Applies the configuration to all boards in the system.
  Sets port configurations, initializes pin states to false, and sends initial intensity settings.
  """
  function set_config(cs::PINController)
    for i in get_bids(cs)
      cconfig = get_cconfig(cs, i)
      cnum = get_cnumbers(cs, i)
      send_spi(cs, i, [REG_CONFIG for _ in cnum], [cconfig for _ in cnum])
      send_spi(cs, i, [REG_PORT_CONFIG_1 for _ in cnum], [0x55 for _ in cnum])
      send_spi(cs, i, [REG_PORT_CONFIG_2 for _ in cnum], [0x55 for _ in cnum])
      send_spi(cs, i, [REG_PORT_CONFIG_3 for _ in cnum], [0x55 for _ in cnum])
      send_spi(cs, i, [REG_PORT_CONFIG_4 for _ in cnum], [0x55 for _ in cnum])
      send_spi(cs, i, [REG_PORT_CONFIG_5 for _ in cnum], [0x55 for _ in cnum])
      send_spi(cs, i, [REG_PORT_CONFIG_6 for _ in cnum], [0x55 for _ in cnum])
      send_spi(cs, i, [REG_PORT_CONFIG_7 for _ in cnum], [0x55 for _ in cnum])
    end
    put_pin_all_state!(cs, false) # all pin states set false
    send_pin_states(cs)           # send all pin states
    send_intensity_states(cs)     # send all intensity states
  end

  """
      matching_antenna_connectors!(cs::PINController, filename::String="data/link_SW_2_board.csv")

  Reads a CSV file to map PIN diodes to antenna connectors and physical coordinates.
  Updates the internal DataFrame with this mapping.
  """
  function matching_antenna_connectors!(cs::PINController, filename::String="data/link_SW_2_board.csv")
    @assert isfile(filename) "File not found"
    ddf = CSV.read(filename, DataFrame)
    cs.df.PINn = fill(missing, nrow(cs.df))
    cs.df.PINn = Union{Missing,Int64}[i for i in cs.df.PINn]
    cs.df.PINconnector = fill(missing, nrow(cs.df))
    cs.df.PINconnector = Union{Missing,String}[i for i in cs.df.PINconnector]
    cs.df.x1 = fill(0.0u"mm", nrow(cs.df))
    cs.df.x1 = typeof(0.0u"mm")[i for i in cs.df.x1]
    cs.df.y1 = fill(0.0u"mm", nrow(cs.df))
    cs.df.y1 = typeof(0.0u"mm")[i for i in cs.df.y1]
    cs.df.θ1 = fill(0.0*radᵃ, nrow(cs.df))
    cs.df.θ1 = typeof(0.0*radᵃ)[i for i in cs.df.θ1]
    cs.df.x2 = fill(0.0u"mm", nrow(cs.df))
    cs.df.x2 = typeof(0.0u"mm")[i for i in cs.df.x2]
    cs.df.y2 = fill(0.0u"mm", nrow(cs.df))
    cs.df.y2 = typeof(0.0u"mm")[i for i in cs.df.y2]
    cs.df.θ2 = fill(0.0*radᵃ, nrow(cs.df))
    cs.df.θ2 = typeof(0.0radᵃ)[i for i in cs.df.θ2]
    cs.df.width = fill(0.0u"mm", nrow(cs.df))
    cs.df.length = typeof(0.0u"mm")[i for i in cs.df.y2]
    for r in eachrow(ddf)
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :PINn] .= parse(Int, r["SW number"][3:end])
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :PINconnector] .= r["Antenna connector"]
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :width] .= r["width(mm)"] * u"mm"
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :length] .= r["length(mm)"] * u"mm"
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :x1] .= r["x1(mm)"]*u"mm"
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :y1] .= r["y1(mm)"]*u"mm"
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :θ1] .= r["theta1(deg)"]*°ᵃ
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :x2] .= r["x2(mm)"]*u"mm"
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :y2] .= r["y2(mm)"]*u"mm"
      cs.df[(cs.df.bid.==r["Board number"]).&&(cs.df.bport.==r["Port number"]), :θ2] .= r["theta2(deg)"] * °ᵃ
    end
  end


  """
      get_board_by_antconnector(cs::PINController, PINconnector::String)

  Retrieves board information associated with a specific antenna connector.
  """
  @inline get_board_by_antconnector(cs::PINController, PINconnector::String) = cs.df[coalesce.(cs.df.PINconnector .== PINconnector, false), :]

  """
      get_board_by_PINn(cs::PINController, PINn::Int)

  Retrieves board information associated with a specific PIN number.
  """
  @inline get_board_by_PINn(cs::PINController, PINn::Int) = cs.df[coalesce.(cs.df.PINn .== PINn, false), :]
  
  """
      select_board(cs::PINController, cs_pin::Integer)

  Activates the Chip Select (CS) line (sets it LOW) for a board.
  """
  select_board(cs::PINController, cs_pin::Integer) = lg_gpio_write(cs.gpio_handle, cs_pin, 0)  # Set CS low
  
  """
      deselect_board(cs::PINController, cs_pin::Integer)

  Deactivates the Chip Select (CS) line (sets it HIGH) for a board.
  """
  deselect_board(cs::PINController, cs_pin::Integer) = lg_gpio_write(cs.gpio_handle, cs_pin, 1)  # Set CS high
  
  """
      get_spis(cs::PINController, bid::Int)

  Retrieves the SPI handle and CS pin associated with a Board ID.
  """
  function get_spis(cs::PINController, bid::Int) 
    df = cs.df[cs.df.bid .== bid, :]
    if isempty(df)
      error("Board ID $bid not found")
    end
    spi_handle = df[1, :spi_handle]
    cs_pin = df[1, :cs_pin]
    return spi_handle, cs_pin
  end
  
  """
      send_spi(cs::PINController, bid::Int, com::Vector{UInt8}, val::Vector{UInt8})

  Sends SPI commands and values to a board.
  
  # Arguments
  - `cs`: PINController instance.
  - `bid`: Board ID.
  - `com`: Vector of commands (one per chip in the chain).
  - `val`: Vector of values (one per chip in the chain).
  """
  function send_spi(cs::PINController, bid::Int, com::Vector{UInt8}, val::Vector{UInt8})
    spi_handle, cs_pin = get_spis(cs, bid)
    tx_buf = Vector{UInt8}()
    for i in get_cnumbers(cs, bid)
      push!(tx_buf, com[i])  # Set the read bit (MSB to 1)
      push!(tx_buf, val[i])  # Dummy byte for reading
    end
    rx_buf = zeros(UInt8, length(tx_buf))
    select_board(cs, cs_pin)
    lg_spi_write(spi_handle, tx_buf, length(tx_buf))
    deselect_board(cs, cs_pin)

    select_board(cs, cs_pin)
    lg_spi_read(spi_handle, rx_buf, length(tx_buf)) < 0 ? error("SPI read failed: $(lg_error_text(result))") : nothing
    deselect_board(cs, cs_pin)
    # println("rx_buf : $(rx_buf)")
    # println("tx_buf : $(tx_buf)")
    @assert rx_buf == tx_buf "rx_buf $(rx_buf) != tx_buf $(tx_buf)"
    reverse(rx_buf)[1:2:end], rx_buf
  end
  
  """
      read_spi(cs::PINController, bid::Int, com::UInt8)

  Reads data from a board via SPI.
  """
  function read_spi(cs::PINController, bid::Int, com::UInt8)
    spi_handle, cs_pin = get_spis(cs, bid)
    tx_buf = Vector{UInt8}()
    for i in get_cnumbers(cs, bid)
      push!(tx_buf, com | 0x80)  # Set the read bit (MSB to 1)
      push!(tx_buf, 0x00)  # Dummy byte for reading
    end
    # Send the command and read the response
    rx_buf = zeros(UInt8, length(tx_buf))
    select_board(cs, cs_pin)
    #result = lg_spi_xfer(board.spi_handle, pointer(tx_buf), pointer(rx_buf), length(tx_buf))
    result = lg_spi_write(spi_handle, tx_buf, length(tx_buf))
    deselect_board(cs, cs_pin)
    select_board(cs, cs_pin)
    #result = lg_spi_xfer(board.spi_handle, pointer(tx_buf), pointer(rx_buf), length(tx_buf))
    result = lg_spi_read(spi_handle, rx_buf, length(tx_buf))
    deselect_board(cs, cs_pin)
    if result < 0
      error("SPI read failed: $(lg_error_text(result))")
    end
    reverse(rx_buf)[1:2:end], rx_buf
  end

  """
      get_spihandle_cspin(cs, bids::Vector{Int})

  Helper function to get SPI handles and CS pins for multiple boards.
  """
  function get_spihandle_cspin(cs, bids::Vector{Int})
    spi_handles = Vector{Int32}(undef, length(bids))
    cs_pins = Vector{Int32}(undef, length(bids))
    for (i, bid) in enumerate(bids)
      spi_handles[i], cs_pins[i] = get_spis(cs, bid)
    end
    spi_handles, cs_pins
  end

  """
      all_send_spi(cs::PINController, com::UInt8, val::UInt8)

  Sends the same command and value to all boards in the system.
  """
  function all_send_spi(cs::PINController, com::UInt8, val::UInt8)
    bids = get_bids(cs)
    spi_handles, cs_pins = get_spihandle_cspin(cs, bids)
    bchipn = [length(get_cnumbers(cs, bid)) for bid in bids]
    tx_buf = Matrix{UInt8}(undef, length(bids), bchipn[1] * 2)
    for i in eachindex(bids)
      for j in range(1, bchipn[i] * 2, step=2)
        tx_buf[i, j] = com
        tx_buf[i, j+1] = val
      end
    end
    rx_buf = similar(tx_buf)

    for i in eachindex(bids)
      select_board(cs, cs_pins[i])
      #result = lg_spi_xfer(board.spi_handle, pointer(tx_buf), pointer(rx_buf), length(tx_buf))
      result = lg_spi_write(spi_handles[i], tx_buf[i, :], length(tx_buf[i, :]))
      deselect_board(cs, cs_pins[i])
      select_board(cs, cs_pins[i])
      #result = lg_spi_xfer(board.spi_handle, pointer(tx_buf), pointer(rx_buf), length(tx_buf))
      result = lg_spi_read(spi_handles[i], rx_buf[i, :], length(tx_buf[i, :]))
      deselect_board(cs, cs_pins[i])
      if result < 0
        @error "SPI read failed: $(i) board : $(lg_error_text(result))"
      end
      # println("rx_buf : $(rx_buf)")
      # println("tx_buf : $(tx_buf)")
      @assert rx_buf[i, :] == tx_buf[i, :] "send data Fail : $(i) board : rx_buf $(rx_buf[i,:]) != tx_buf $(tx_buf[i,:])\n"
      #reverse(rx_buf)[1:2:end], rx_buf
    end
  end
  
  """
      all_read_spi(cs::PINController, bid::Int, com::UInt8)

  Reads from SPI for a specific board.
  (Note: Function name implies 'all' but implementation takes a single `bid`. This might be a misnomer or intended for a different pattern).
  """
  function all_read_spi(cs::PINController, bid::Int, com::UInt8)
    spi_handle, cs_pin = get_spis(cs, bid)
    tx_buf = Vector{UInt8}()
    for i in get_cnumbers(cs, bid)
      push!(tx_buf, com | 0x80)  # Set the read bit (MSB to 1)
      push!(tx_buf, 0x00)  # Dummy byte for reading
    end
    # Send the command and read the response
    rx_buf = zeros(UInt8, length(tx_buf))
    select_board(cs, cs_pin)
    #result = lg_spi_xfer(board.spi_handle, pointer(tx_buf), pointer(rx_buf), length(tx_buf))
    result = lg_spi_write(spi_handle, tx_buf, length(tx_buf))
    deselect_board(cs, cs_pin)
    select_board(cs, cs_pin)
    #result = lg_spi_xfer(board.spi_handle, pointer(tx_buf), pointer(rx_buf), length(tx_buf))
    result = lg_spi_read(spi_handle, rx_buf, length(tx_buf))
    deselect_board(cs, cs_pin)
    if result < 0
      error("SPI read failed: $(lg_error_text(result))")
    end
    reverse(rx_buf)[1:2:end], rx_buf
  end

  """
      get_bids(cs::PINController)

  Returns a list of unique Board IDs in the system.
  """
  get_bids(cs::PINController) = unique(cs.df[:, :bid])

  """
      get_cconfig(cs::PINController, bid::Int)

  Retrieves the configuration byte for a specific board.
  """
  get_cconfig(cs::PINController, bid::Int) = cs.df[coalesce.(cs.df.bid .== bid, false).&coalesce.(cs.df.cpin_enable .== true, false), :cconfig][1, 1]

  """
      get_cnumbers(cs::PINController, bid::Int)

  Retrieves the sorted list of unique chip IDs for a specific board.
  """
  get_cnumbers(cs::PINController, bid::Int) = sort!(unique(cs.df[coalesce.(cs.df.bid .== bid, false).&coalesce.(cs.df.cpin_enable .== true, false), :cid]))

  """
      get_board(cs::PINController, bid::Int)

  Retrieves the DataFrame rows corresponding to a specific board.
  """
  get_board(cs::PINController, bid::Int) = cs.df[coalesce.(cs.df.bid .== bid, false).&coalesce.(cs.df.cpin_enable .== true, false), :]

  """
      change_pid_states!(cs::PINController, pids::Union{Vector,UnitRange}, states::Vector{Bool})

  Updates the state of specific System PIDs in the internal DataFrame.
  """
  @inline function change_pid_states!(cs::PINController, pids::Union{Vector,UnitRange}, states::Vector{Bool}) 
    @assert length(pids) == length(states) "Length of pids and states must be the same"
    for (pid, state) in zip(pids, states)
      cs.df[coalesce.(cs.df[:,:pid] .== pid, false), :cpin_state] .= state
    end
  end

  """
      getbycid(cs::PINController, cid::Int)

  Retrieves DataFrame rows for a specific Chip ID.
  """
  @inline getbycid(cs::PINController, cid::Int) = cs.df[coalesce.(cs.df.cid .== cid, false).&coalesce.(cs.df.cpin_enable .== true, false), :]

  """
      getbybid(cs::PINController, bid::Int)

  Retrieves DataFrame rows for a specific Board ID.
  """
  @inline getbybid(cs::PINController, bid::Int) = cs.df[coalesce.(cs.df.bid .== bid, false).&coalesce.(cs.df.cpin_enable .== true, false), :]

  """
      getbybport(cs::PINController, bport::Int)

  Retrieves DataFrame rows for a specific Board Port.
  """
  @inline getbybport(cs::PINController, bport::Int) = cs.df[coalesce.(cs.df.bport .== bport, false).&coalesce.(cs.df.cpin_enable .== true, false), :]

  """
      vector_to_uint8(vc::Vector)

  Converts a boolean vector (up to 8 elements) into a UInt8 byte.
  """
  @inline function vector_to_uint8(vc::Vector)
    n = length(vc)
    #@assert 1 <= n <= 8 "Vector must have 1 to 8 elements"
    #@assert all(iszero, vc) "All elements must be zero"
    vc = reinterpret(Bool, vc)
    if n < 8
      vc = vcat(vc, zeros(Bool, 8 - n))
    end
    result = UInt8(0)
    for (i, val) in enumerate(reverse(vc))
      result |= UInt8(val) << (8 - i)
    end
    return result
  end

  """
      put_pin_state_bybid!(cs::PINController, bid::Int, bports::Vector{Int}, state::Vector{Bool})

  Updates the state of pins specified by Board ID and Board Ports.
  """
  function put_pin_state_bybid!(cs::PINController, bid::Int, bports::Vector{Int}, state::Vector{Bool})
    @assert length(bid) == length(state) "bid"
    for (i, b) in enumerate(bports)
    cs.df[coalesce.(cs.df[:, :bid] .== bid, false).&&coalesce.(cs.df[:, :bport] .== b, false), :cpin_state] .= state[i]
    end
  end

  """
      get_active_pins(cs::PINController)

  Returns a DataFrame of all pins that are enabled and have an assigned PIN number.
  """
  @inline get_active_pins(cs::PINController) = cs.df[(cs.df.cpin_enable .== true) .&& (cs.df.PINn .!== missing), :]

  """
      put_pin_all_state!(cs::PINController, state::Bool)

  Sets the state of ALL pins in the system to the specified value.
  """
  function put_pin_all_state!(cs::PINController, state::Bool) 
    cs.df[:, :cpin_state] .= state
  end

  """
      put_pin_state!(cs::PINController, PINs::Vector{Int}, state::Vector{Bool})

  Updates the state of specific pins identified by their PIN numbers.
  """
  function put_pin_state!(cs::PINController, PINs::Vector{Int}, state::Vector{Bool})
    @assert length(PINs) == length(state) "pid"
    for (i, p) in enumerate(PINs)
      cs.df[coalesce.(cs.df[:, :PINs] .== p, false), :cpin_state] .= state[i]
    end
  end

  """
      get_pin_state(cs::PINController, PINs::Vector{Int})

  Retrieves the current state of specific pins.
  """
  function get_pin_state(cs::PINController, PINs::Vector{Int})
    @assert !isempty(PINs) "PINs"
    return cs.df[coalesce.(cs.df[:, :PINs] .== pid, false), :cpin_state]
  end

  """
      send_pin_states(cs::PINController, check::Bool=false)

  Transmits the current pin states from the internal DataFrame to the hardware.
  If `check` is true, it reads back the states to verify.
  """
  function send_pin_states(cs::PINController, check::Bool=false)
    #switches = BitVector(switches)
    for bid in get_bids(cs)
      #df = sort!(cs.df[(cs.df.bid .== 1),:], [:cid, :cport])
      df = cs.df[(cs.df.bid.==bid), :]
      s = [1, 9, 17, 25] .+ 3
      e = [8, 16, 24, 28] .+ 3
      for (i, c) in enumerate([REG_4_11, REG_12_19, REG_20_27, REG_28_31])
        cmds = UInt8[]
        vals = UInt8[]
        for cid = reverse(unique(df[:,:cid])) # 11:1
          push!(cmds, c)
          push!(vals, vector_to_uint8([df[coalesce.(df.cport .== k, false).&& coalesce.(df.cid .== cid, false), :cpin_state][1] for k = s[i]:e[i]]))
        end
        send_spi(cs, bid, cmds, vals)
      end
    end
    if check
      for bid in get_bids(cs)
        df = cs.df[(cs.df.bid.==bid), :]
        s = [1, 9, 17, 25] .+ 3
        e = [8, 16, 24, 28] .+ 3
        for (i, c) in enumerate([REG_4_11, REG_12_19, REG_20_27, REG_28_31])
          cmds = UInt8[]
          vals = UInt8[]
          for cid = reverse(unique(df[:, :cid])) # 11:1
            push!(cmds, c)
            push!(vals, vector_to_uint8([df[coalesce.(df.cport .== k, false).&&coalesce.(df.cid .== cid, false), :cpin_state][1] for k = s[i]:e[i]]))
          end
          r,_ = read_spi(cs, bid, cmds)
          println("$(bid) : $(i), : $(vals)")
          println("$(bid) : $(i), : $(reverse(r))")
        end
      end
    end
    nothing
  end

  """
      put_intensity_bybid!(cs::PINController, bid::Int, bport::Int, intensity::Int)

  Sets the intensity for a specific board port.
  Intensity is 0 (max) to 15 (min).
  """
  @inline function put_intensity_bybid!(cs::PINController, bid::Int, bport::Int, intensity::Int) 
    cs.df[coalesce.(cs.df.bid .== bid, false).&coalesce.(cs.df.bport .== bport, false), :cpin_intensity] .= intensity
  end

  """
      put_intensity!(cs::PINController, pinn::Vector{Int}, intensity::Vector{Int})

  Sets the intensity for specific PIN numbers.
  Intensity is 0 (max) to 15 (min).
  """
  @inline function put_intensity!(cs::PINController, pinn::Vector{Int}, intensity::Vector{Int})
    @assert all(x -> (x >= filter(!ismissing, sort(cs.df[:, :PINn]))[1]) && (x <= filter(!ismissing, sort(cs.df[:, :PINn], rev=true))[1]), pinn) "Intensity must be between 1 and 16"
    @assert all(x -> x >= 0 && x <= 15, intensity) "Intensity must be between 1 and 16"
    for (p, i) in zip(pinn, intensity)
      print(p," ",i)
      cs.df[coalesce.(cs.df.PINn .== p, false), :cpin_intensity] .= i
      cs.df[coalesce.(cs.df.PINn .== p, false), :cpin_current] .= 936u"kΩ" / cs.riset / i * u"mA"
    end
    nothing
  end

  """
  set_display_test_mode(cs::PINController, enable::Bool)

  Enables or disables the Display Test Mode for all chips in the system.
  When enabled, all LEDs are turned on at maximum intensity (15/16 or 31/32 duty cycle).
  This overrides all other controls.

  # Arguments
  - `cs::PINController`: The PIN controller system.
  - `enable::Bool`: `true` to enable test mode, `false` for normal operation.
  """
  function set_display_test_mode(cs::PINController, enable::Bool)
    val = enable ? 0x01 : 0x00
    all_send_spi(cs, REG_DISPLAY_TEST, val)
  end

  """
  set_current_mode(cs::PINController, mode::Symbol)

  Sets the current control mode for all chips in the system.

  # Arguments
  - `cs::PINController`: The PIN controller system.
  - `mode::Symbol`: `:global` for global current control, `:individual` for individual current control.
  """
  function set_current_mode(cs::PINController, mode::Symbol)
    if mode == :global
      # Clear bit D6 for Global Current
      # We need to read the current config to preserve other bits, but since we don't track it perfectly,
      # we will assume the standard config and just modify the relevant bit.
      # Standard config in put_board! is CONFIG_NORMAL | CONFIG_INDIVIDUAL_CURRENT | CONFIG_DETECT_DISABLED
      # We will reconstruct the base config.
      base_config = CONFIG_NORMAL | CONFIG_DETECT_DISABLED
      # For global, we do NOT add CONFIG_INDIVIDUAL_CURRENT (0x40)
      new_config = base_config | CONFIG_GLOBAL_CURRENT
    elseif mode == :individual
      base_config = CONFIG_NORMAL | CONFIG_DETECT_DISABLED
      new_config = base_config | CONFIG_INDIVIDUAL_CURRENT
    else
      error("Invalid mode. Use :global or :individual")
    end

    # Update local DataFrame to reflect the change (optional but good for consistency)
    cs.df[!, :cconfig] .= new_config
    
    # Send to all chips
    all_send_spi(cs, REG_CONFIG, new_config)
  end

  """
  set_global_current(cs::PINController, current::Int)

  Sets the global current level for all chips when in Global Current mode.

  # Arguments
  - `cs::PINController`: The PIN controller system.
  - `current::Int`: Current level from 0 (min) to 15 (max).
  """
  function set_global_current(cs::PINController, current::Int)
    @assert 0 <= current <= 15 "Current must be between 0 and 15"
    # The register value is just the current level (0x00 to 0x0F)
    all_send_spi(cs, REG_GLOBAL_CURRENT, UInt8(current))
  end

  """
  send_spi_selective_chips(cs::PINController, bid::Int, target_cids::Vector{Int}, coms::Vector{UInt8}, vals::Vector{UInt8})

  Sends commands to specific chips in the daisy chain of a single board, sending No-Op to others.

  # Arguments
  - `cs::PINController`: The PIN controller system.
  - `bid::Int`: The Board ID containing the daisy-chained chips.
  - `target_cids::Vector{Int}`: List of Chip IDs (1-11) to update.
  - `coms::Vector{UInt8}`: Commands for the target chips.
  - `vals::Vector{UInt8}`: Values for the target chips.
  """
  function send_spi_selective_chips(cs::PINController, bid::Int, target_cids::Vector{Int}, coms::Vector{UInt8}, vals::Vector{UInt8})
    @assert length(target_cids) == length(coms) == length(vals) "Length of cids, coms, and vals must match"
    
    spi_handle, cs_pin = get_spis(cs, bid)
    all_cids = get_cnumbers(cs, bid) # Should be 1 to 11
    
    tx_buf = Vector{UInt8}()
    
    # Create a dictionary for easy lookup of targets
    target_map = Dict(zip(target_cids, zip(coms, vals)))
    
    for cid in all_cids
      if haskey(target_map, cid)
        c, v = target_map[cid]
        push!(tx_buf, c)
        push!(tx_buf, v)
      else
        # Send No-Op
        push!(tx_buf, REG_NO_OP)
        push!(tx_buf, 0x00) # Dummy value for No-Op
      end
    end
    
    select_board(cs, cs_pin)
    lg_spi_write(spi_handle, tx_buf, length(tx_buf))
    deselect_board(cs, cs_pin)
  end

  """
      send_intensity_states(cs::PINController)

  Transmits the current intensity settings from the internal DataFrame to the hardware.
  """
  function send_intensity_states(cs::PINController)
    for bid in get_bids(cs)
      df = cs.df[(cs.df.bid.==bid), :]
      regs = [REG_CURRENT_054, REG_CURRENT_076, REG_CURRENT_098, REG_CURRENT_0BA, REG_CURRENT_0DC, REG_CURRENT_0FE, REG_CURRENT_110, REG_CURRENT_132, REG_CURRENT_154, REG_CURRENT_176, REG_CURRENT_198, REG_CURRENT_1BA, REG_CURRENT_1DC, REG_CURRENT_1FE]
      cports = 4:2:31
      for (c,r) in zip(cports, regs)
        cmds = UInt8[]
        vals = UInt8[]
        for cid in reverse(unique(df[:, :cid])) # 11:1
          push!(cmds, r)
        t = [df[coalesce.(df.cport .== k, false).&&coalesce.(df.cid .== cid, false), :cpin_intensity][1] for k = c:(c+1)]
          push!(vals, t[2] << 4 | t[1])
        end
        send_spi(cs, bid, cmds, vals)
      end
    end
  end

  """
      example()

  Demonstration function showing how to initialize the system, add boards, and control pins.
  """
  function example()
    # Open GPIO chip
    gpio_handle = Int64(lg_gpiochip_open(0))
    if gpio_handle < 0
      error("Failed to open GPIO chip: $(lg_error_text(gpio_handle))")
    end
    # Open SPI0 handles
    spi0_handle = lg_spi_open(0, 0, 15_000_000, 0)
    if spi0_handle < 0
      error("Failed to open SPI0 : $(lg_error_text(gpio_handle))")
    end
    spi0_cs0_pin = 8 # GPIO pin 8 is used for SPI0_CS0
    lg_set_gpio_output(gpio_handle, spi0_cs0_pin)
    spi0_cs1_pin = 7 # GPIO pin 7 is used for SPI0_CS1
    lg_set_gpio_output(gpio_handle, spi0_cs1_pin)
    spi0_cs2_pin = 25 # GPIO pin 25 is used for SPI0_CS2
    lg_set_gpio_output(gpio_handle, spi0_cs2_pin)

    # Open SPI1 handles 
    spi1_handle = lg_spi_open(1, 0, 15_000_000, 0)
    if spi1_handle < 0
      error("Failed to open SPI1 : $(lg_error_text(gpio_handle))")
    end
    spi1_cs0_pin = 16 # GPIO pin 16 is used for SPI1_CS0
    lg_set_gpio_output(gpio_handle, spi1_cs0_pin)
    spi1_cs1_pin = 6 # GPIO pin 6 is used for SPI1_CS1
    lg_set_gpio_output(gpio_handle, spi1_cs1_pin)
    spi1_cs2_pin = 5 # GPIO pin 5 is used for SPI1_CS2
    lg_set_gpio_output(gpio_handle, spi1_cs2_pin)

    
    cs = PINController(gpio_handle)
    put_board!(cs, 1, spi0_handle, spi0_cs0_pin) # add board 1
    put_board!(cs, 2, spi0_handle, spi0_cs1_pin) # add board 2
    put_board!(cs, 3, spi0_handle, spi0_cs2_pin) # add board 3
    put_board!(cs, 4, spi1_handle, spi1_cs0_pin) # add board 4
    put_board!(cs, 5, spi1_handle, spi1_cs1_pin) # add board 5
    put_board!(cs, 6, spi1_handle, spi1_cs2_pin) # add board 6
    matching_antenna_connectors!(cs) # add PIN informations
    @time set_config(cs) # set configuration for all boards

    lg_spi_close(spi0_handle)
    lg_spi_close(spi1_handle)
    lg_gpiochip_close(gpio_handle)

  end

  function __init__()
    precompile(Tuple{typeof(PINController), Int, Unitful.ElectricalResistance})
    precompile(Tuple{typeof(put_board!), PINController, Int, Int, Int})
    precompile(Tuple{typeof(set_config), PINController})
    precompile(Tuple{typeof(matching_antenna_connectors!), PINController, String})
    precompile(Tuple{typeof(get_board_by_antconnector), PINController, String})
    precompile(Tuple{typeof(get_board_by_PINn), PINController, Int})
    precompile(Tuple{typeof(select_board), PINController, Int})
    precompile(Tuple{typeof(deselect_board), PINController, Int})
    precompile(Tuple{typeof(get_spis), PINController, Int})
    precompile(Tuple{typeof(send_spi), PINController, Int, Vector{UInt8}, Vector{UInt8}})
    precompile(Tuple{typeof(read_spi), PINController, Int, UInt8})
    precompile(Tuple{typeof(get_bids), PINController})
    precompile(Tuple{typeof(get_cconfig), PINController, Int})
    precompile(Tuple{typeof(get_cnumbers), PINController, Int})
    precompile(Tuple{typeof(get_board), PINController, Int})
    precompile(Tuple{typeof(change_pid_states!), PINController, Vector, Vector})
    precompile(Tuple{typeof(getbycid), PINController, Int})
    precompile(Tuple{typeof(getbybid), PINController, Int})
    precompile(Tuple{typeof(getbybport), PINController, Int})
    precompile(Tuple{typeof(vector_to_uint8), Vector})
    precompile(Tuple{typeof(put_pin_state_bybid!), PINController, Int, Int, Bool})
    precompile(Tuple{typeof(get_active_pins), PINController})
    precompile(Tuple{typeof(put_pin_all_state!), PINController, Bool})
    precompile(Tuple{typeof(put_pin_state!), PINController, Vector{Int}, Vector{Bool}})
    precompile(Tuple{typeof(get_pin_state), PINController, Vector{Int}})
    precompile(Tuple{typeof(send_pin_states), PINController})
    precompile(Tuple{typeof(put_intensity_bybid!), PINController, Int, Int, Int})
    precompile(Tuple{typeof(put_intensity!), PINController, Vector{Int}, Vector{Int}})
    precompile(Tuple{typeof(send_intensity_states), PINController})
  end

  function test()
    bids = [1, 2, 3, 4, 5, 6]
    spi_handles = [1, 1, 1, 2, 2, 2]
    cspins = [8, 7, 25, 16, 6, 5]
  end
end
