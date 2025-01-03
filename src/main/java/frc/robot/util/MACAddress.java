package frc.robot.util;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

/**
 * Class to deal with robot MAC Addresses
 *
 * @author Alex Stedman
 */
@SuppressWarnings("EqualsBetweenInconvertibleTypes")
public class MACAddress {
  private final byte[] address;

  /**
   * Create MAC Address
   *
   * @param addresses bytes of address
   */
  public MACAddress(byte... addresses) {
    address = addresses;
  }

  /**
   * get the bytes of the address
   *
   * @return the bytes of the address
   */
  public byte[] getAddress() {
    return address;
  }

  /**
   * does this MAC Address equal another or do the bytes match?
   *
   * @param obj thing to compare
   * @return if they should be the same
   */
  public boolean matches(Object obj) {
    if (obj instanceof byte[]) return Arrays.equals(address, (byte[]) obj);
    if (obj instanceof MACAddress) return matches(((MACAddress) obj).getAddress());
    return false;
  }

  /**
   * does this mac address exist on the network
   *
   * @return if this mac address exists on the network
   */
  public boolean exists() {
    try {
      for (byte[] b : getAll()) {
        if (matches(b)) return true;
      }
      return false;
    } catch (IOException ignored) {
      return false;
    }
  }

  /**
   * get all mac addresses on the network
   *
   * @return a List of the byte addresses
   * @throws IOException if network cannot be found
   */
  public static List<byte[]> getAll() throws IOException {
    List<byte[]> macAddresses = new ArrayList<>();

    Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

    NetworkInterface networkInterface;
    while (networkInterfaces.hasMoreElements()) {
      networkInterface = networkInterfaces.nextElement();

      byte[] address = networkInterface.getHardwareAddress();
      if (address != null) macAddresses.add(address);
    }

    return macAddresses;
  }

  /**
   * Shorthand create a simplified addres
   *
   * @param a 4th byte term as int
   * @param b 5th byte term as int
   * @param c 6th byte term as int
   * @return new MACAddress
   */
  public static MACAddress of(int a, int b, int c) {
    return new MACAddress((byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) a, (byte) b, (byte) c);
  }
}
