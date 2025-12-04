"""
Nœud de filtrage et prétraitement des données sonar.
Implémente normalisation, TVG, égalisation d'histogramme et SO-CFAR.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame
import numpy as np
from scipy import ndimage
try:
    import cv2
except ImportError:  # Provide a minimal fallback to avoid import-time crash
    cv2 = None


class TraitementNode(Node):
    """Applique filtres avancés sur frames sonar brutes."""
    
    def __init__(self):
        super().__init__('traitement_node')
        
        # Paramètres de normalisation et TVG
        self.declare_parameter('enable_histogram_eq', True)
        self.declare_parameter('enable_tvg_correction', True)
        self.declare_parameter('tvg_alpha', 0.0002)  # Coefficient d'atténuation
        self.declare_parameter('tvg_spreading_loss', 20.0)  # Perte par étalement (dB)
        
        # Paramètres de filtrage de base
        self.declare_parameter('enable_median', True)
        self.declare_parameter('median_kernel', 3)
        self.declare_parameter('enable_gaussian', True)
        self.declare_parameter('gaussian_sigma', 1.0)
        # Filtre bilatéral (lissage tout en préservant bords)
        self.declare_parameter('enable_bilateral', False)
        self.declare_parameter('bilateral_d', 5)            # diamètre voisinage
        self.declare_parameter('bilateral_sigma_color', 25.0)
        self.declare_parameter('bilateral_sigma_space', 5.0)

        # Morphologie: top-hat (extrait petits éléments brillants)
        self.declare_parameter('enable_tophat', False)
        self.declare_parameter('tophat_kernel', 5)

        # Détection de blobs: LoG / DoG
        self.declare_parameter('enable_log_enhance', False)
        self.declare_parameter('log_sigma', 1.0)
        self.declare_parameter('enable_dog_enhance', False)
        self.declare_parameter('dog_sigma1', 1.0)
        self.declare_parameter('dog_sigma2', 2.0)

        # Filtre adapté (matched filter) avec PSF gaussien
        self.declare_parameter('enable_matched_filter', False)
        self.declare_parameter('mf_sigma', 1.2)
        self.declare_parameter('mf_kernel_size', 9)
        
        # Paramètres SO-CFAR
        self.declare_parameter('enable_so_cfar', True)
        self.declare_parameter('cfar_guard_cells', 2)  # Cellules de garde
        self.declare_parameter('cfar_window_size', 10)  # Taille fenêtre référence
        self.declare_parameter('cfar_alpha', 3.0)  # Facteur seuil (taux fausse alarme)
        
        # Paramètres seuillage adaptatif final
        self.declare_parameter('enable_adaptive_threshold', True)
        self.declare_parameter('adt_block_size', 15)  # Taille bloc (doit être impair)
        self.declare_parameter('adt_c', 2)  # Constante soustraite à la moyenne
        
        # Subscription
        self.subscription = self.create_subscription(
            Frame,
            '/docking/sonar/raw',
            self.frame_callback,
            10
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/filtered', 10)
        
        self.get_logger().info('Traitement node démarré (TVG + SO-CFAR enabled)')
    
    def tvg_correction(self, img: np.ndarray, ranges: np.ndarray) -> np.ndarray:
        """
        Correction Time Varying Gain (TVG).
        Compense l'atténuation due à l'étalement géométrique et l'absorption.
        """
        # Modèle de perte: spreading loss + absorption
        spreading_loss_db = self.get_parameter('tvg_spreading_loss').value
        alpha = self.get_parameter('tvg_alpha').value
        
        # Calcul du gain en dB pour chaque range bin
        # Spreading loss: 20*log10(r) ou 10*log10(r) selon géométrie
        # Absorption: alpha * r (Neper/m converti en dB)
        gain_db = spreading_loss_db * np.log10(ranges + 1e-6) + (alpha * ranges * 8.686)
        
        # Conversion dB -> linéaire
        gain_linear = 10 ** (gain_db / 20.0)
        
        # Application du gain (broadcast sur tous les bearings)
        corrected = img.astype(np.float32) * gain_linear[np.newaxis, :]
        
        return np.clip(corrected, 0, 255).astype(np.uint8)
    
    def histogram_equalization(self, img: np.ndarray) -> np.ndarray:
        """
        Égalisation d'histogramme adaptative (CLAHE).
        Améliore le contraste local tout en limitant l'amplification du bruit.
        """
        if cv2 is None:
            # Fallback: simple histogram equalization with NumPy
            hist, bins = np.histogram(img.flatten(), 256, [0,256])
            cdf = hist.cumsum()
            cdf_masked = np.ma.masked_equal(cdf, 0)
            cdf_masked = (cdf_masked - cdf_masked.min())*255/(cdf_masked.max()-cdf_masked.min())
            cdf = np.ma.filled(cdf_masked, 0).astype('uint8')
            return cdf[img]
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(img)

    def bilateral_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre bilatéral via OpenCV (préserve les bords)."""
        if cv2 is None:
            return img  # no-op if OpenCV missing
        d = int(self.get_parameter('bilateral_d').value)
        sigma_color = float(self.get_parameter('bilateral_sigma_color').value)
        sigma_space = float(self.get_parameter('bilateral_sigma_space').value)
        return cv2.bilateralFilter(img, d, sigma_color, sigma_space)

    def tophat_filter(self, img: np.ndarray) -> np.ndarray:
        """White top-hat morphologique pour extraire petits éléments brillants."""
        if cv2 is None:
            return img
        k = int(self.get_parameter('tophat_kernel').value)
        if k < 1:
            return img
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        return cv2.morphologyEx(img, cv2.MORPH_TOPHAT, kernel)

    def log_enhance(self, img: np.ndarray) -> np.ndarray:
        """Laplacian of Gaussian (approx) pour renforcer blobs."""
        if cv2 is None:
            return ndimage.laplace(ndimage.gaussian_filter(img, sigma=float(self.get_parameter('log_sigma').value)))
        sigma = float(self.get_parameter('log_sigma').value)
        g = cv2.GaussianBlur(img, (0, 0), sigma)
        lap = cv2.Laplacian(g, cv2.CV_16S, ksize=3)
        lap_abs = cv2.convertScaleAbs(lap)
        enhanced = cv2.addWeighted(img, 1.0, lap_abs, 1.0, 0)
        return enhanced

    def dog_enhance(self, img: np.ndarray) -> np.ndarray:
        """Difference of Gaussian pour accentuer pics multi-échelle."""
        if cv2 is None:
            s1 = float(self.get_parameter('dog_sigma1').value)
            s2 = float(self.get_parameter('dog_sigma2').value)
            g1 = ndimage.gaussian_filter(img, s1)
            g2 = ndimage.gaussian_filter(img, s2)
            dog = g1 - g2
            dog = (dog - dog.min()) / (dog.max() - dog.min() + 1e-6) * 255.0
            return dog.astype(np.uint8)
        s1 = float(self.get_parameter('dog_sigma1').value)
        s2 = float(self.get_parameter('dog_sigma2').value)
        g1 = cv2.GaussianBlur(img, (0, 0), s1)
        g2 = cv2.GaussianBlur(img, (0, 0), s2)
        dog = cv2.subtract(g1, g2)
        dog = cv2.normalize(dog, None, 0, 255, cv2.NORM_MINMAX)
        return dog.astype(np.uint8)

    def matched_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre adapté (corrélation) avec PSF gaussien 2D."""
        # Works without cv2 by using ndimage.convolve
        sigma = float(self.get_parameter('mf_sigma').value)
        ksize = int(self.get_parameter('mf_kernel_size').value)
        ksize = max(3, ksize | 1)  # impair
        ax = np.arange(-(ksize//2), ksize//2 + 1)
        xx, yy = np.meshgrid(ax, ax)
        psf = np.exp(-(xx**2 + yy**2) / (2 * sigma**2))
        psf /= psf.sum()
        if cv2 is None:
            out = ndimage.convolve(img.astype(np.float32), psf, mode='reflect')
        else:
            out = cv2.filter2D(img.astype(np.float32), -1, psf)
        return np.clip(out, 0, 255).astype(np.uint8)
    
    def so_cfar_detector(self, img: np.ndarray) -> np.ndarray:
        """
        SO-CFAR (Smallest Of - Constant False Alarm Rate).
        Détection adaptative de cibles avec rejet du bruit de fond variable.
        
        Principe:
        - Fenêtre glissante autour de chaque pixel (cellule sous test)
        - Division en 2 sous-fenêtres (avant/arrière le long du range)
        - Calcul de la moyenne dans chaque sous-fenêtre
        - Sélection de la PLUS PETITE moyenne (robuste aux cibles voisines)
        - Seuil local: T = alpha * mu_min
        - Décision: pixel > T => cible potentielle
        """
        guard = self.get_parameter('cfar_guard_cells').value
        window = self.get_parameter('cfar_window_size').value
        alpha = self.get_parameter('cfar_alpha').value
        
        bearing_count, range_count = img.shape
        output = np.zeros_like(img, dtype=np.uint8)
        
        # Conversion en float pour calculs
        img_float = img.astype(np.float32)
        
        # Parcours de l'image (éviter les bords)
        margin = guard + window
        
        for b in range(bearing_count):
            for r in range(margin, range_count - margin):
                # Cellule sous test
                cut = img_float[b, r]
                
                # Fenêtre avant (leading)
                leading_start = r - guard - window
                leading_end = r - guard
                leading_window = img_float[b, leading_start:leading_end]
                
                # Fenêtre arrière (trailing)
                trailing_start = r + guard + 1
                trailing_end = r + guard + window + 1
                trailing_window = img_float[b, trailing_start:trailing_end]
                
                # Moyennes des deux fenêtres
                mu_leading = np.mean(leading_window) if leading_window.size > 0 else 0
                mu_trailing = np.mean(trailing_window) if trailing_window.size > 0 else 0
                
                # SO-CFAR: sélection de la plus petite moyenne
                mu_min = min(mu_leading, mu_trailing)
                
                # Seuil adaptatif
                threshold = alpha * mu_min
                
                # Décision
                if cut > threshold:
                    output[b, r] = 255  # Cible détectée
                else:
                    output[b, r] = 0
        
        return output
    
    def adaptive_threshold(self, img: np.ndarray) -> np.ndarray:
        """
        Seuillage adaptatif local (ADT).
        Binarisation finale pour ne garder que les structures pertinentes.
        """
        block_size = self.get_parameter('adt_block_size').value
        c = self.get_parameter('adt_c').value
        
        # Assurer que block_size est impair
        if block_size % 2 == 0:
            block_size += 1
        
        return cv2.adaptiveThreshold(
            img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, block_size, c
        )
    
    def frame_callback(self, msg: Frame):
        """Traite une frame sonar."""
        # Reconstruction image 2D
        img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.bearing_count, msg.range_count)
        )
        
        filtered = img.copy()
        
        # 1. Correction TVG (Time Varying Gain)
        if self.get_parameter('enable_tvg_correction').value:
            ranges = np.linspace(msg.min_range, msg.max_range, msg.range_count)
            filtered = self.tvg_correction(filtered, ranges)
        
        # 2. Égalisation d'histogramme (CLAHE)
        if self.get_parameter('enable_histogram_eq').value:
            filtered = self.histogram_equalization(filtered)
        
        # 3. Filtrage médian (réduction bruit impulsionnel)
        if self.get_parameter('enable_median').value:
            kernel = self.get_parameter('median_kernel').value
            filtered = ndimage.median_filter(filtered, size=kernel)
        
        # 4. Filtrage gaussien (lissage)
        if self.get_parameter('enable_gaussian').value:
            sigma = self.get_parameter('gaussian_sigma').value
            filtered = ndimage.gaussian_filter(filtered, sigma=sigma)

        # 5. Filtre bilatéral
        if self.get_parameter('enable_bilateral').value:
            filtered = self.bilateral_filter(filtered)

        # 6. Morphologie top-hat
        if self.get_parameter('enable_tophat').value:
            filtered = self.tophat_filter(filtered)

        # 7. Enhancement LoG / DoG / Matched Filter
        if self.get_parameter('enable_log_enhance').value:
            filtered = self.log_enhance(filtered)
        if self.get_parameter('enable_dog_enhance').value:
            filtered = self.dog_enhance(filtered)
        if self.get_parameter('enable_matched_filter').value:
            filtered = self.matched_filter(filtered)
        
        # 5. SO-CFAR (détection adaptative de cibles)
        if self.get_parameter('enable_so_cfar').value:
            filtered = self.so_cfar_detector(filtered)
        
        # 6. Seuillage adaptatif final (ADT)
        if self.get_parameter('enable_adaptive_threshold').value:
            filtered = self.adaptive_threshold(filtered)
        
        # Publication
        out_msg = Frame()
        out_msg.header = msg.header
        out_msg.range_count = msg.range_count
        out_msg.bearing_count = msg.bearing_count
        out_msg.range_resolution = msg.range_resolution
        out_msg.bearing_resolution = msg.bearing_resolution
        out_msg.min_range = msg.min_range
        out_msg.max_range = msg.max_range
        out_msg.intensities = filtered.flatten().tolist()
        out_msg.sound_speed = msg.sound_speed
        out_msg.gain = msg.gain
        
        self.publisher_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TraitementNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
