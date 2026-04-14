module Main where

import Data.Array.Base
import Data.Array.IArray
import Data.Array.Unboxed

import qualified Data.ByteString.Lazy as BL

import Data.Bits

import Data.List (isSuffixOf)

import Data.Word

import System.Environment

import System.FilePath

reverseBitsSlow :: Word8 -> Word8
reverseBitsSlow x = foldl setBit 0 [0..7]
  where
    setBit acc i
      | (x `shiftR` i) .&. 1 == 1 = acc .|. (1 `shiftL` (7 - i))
      | otherwise = acc

tab :: UArray Word8 Word8
tab = array (minBound, maxBound)
            [(i, reverseBitsSlow i) | i <- [minBound..maxBound]]

reverseBits :: Word8 -> Word8
reverseBits = unsafeAt tab . fromIntegral

handleArgs :: [String] -> IO ()
handleArgs [fp]
    | ".rbf" `isSuffixOf` fp = do
        i <- BL.readFile fp
        let rb = BL.map reverseBits i
        BL.writeFile (replaceExtension fp "rbf_r") rb
handleArgs _ = putStrLn "no"

main :: IO ()
main = getArgs >>= handleArgs
